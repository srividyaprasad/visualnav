import os
import pickle
from PIL import Image
import argparse
import tqdm
import yaml
import io

# ROS 2 specific
from rosbag2_py import SequentialReader
from rosbag2_py import StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# utils
from vint_train.process_data.process_data_utils import *


def load_ros2_bag(bag_path, topics):
    """
    Load and extract messages from a ROS 2 bag.
    Returns a dictionary: {topic_name: list_of_msgs}
    """
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    data = {topic: [] for topic in topics if topic in type_map}

    while reader.has_next():
        topic, msg_data, timestamp = reader.read_next()
        if topic in data:
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(msg_data, msg_type)
            data[topic].append((timestamp, msg))
    return data


def main(args: argparse.Namespace):
    # load the config file
    with open("vint_train/process_data/process_bags_config.yaml", "r") as f:
        config = yaml.load(f, Loader=yaml.FullLoader)

    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)

    # Get all .db3 files (ros2 bags)
    bag_files = []
    for root, dirs, files in os.walk(args.input_dir):
        for file in files:
            if file.endswith(".db3"):
                bag_files.append(root)  # use folder path (ROS 2 bag is a folder with metadata)

    bag_files = list(set(bag_files))  # remove duplicate folder paths
    if args.num_trajs >= 0:
        bag_files = bag_files[: args.num_trajs]

    for bag_path in tqdm.tqdm(bag_files, desc="Bags processed"):
        try:
            bag_data = load_ros2_bag(
                bag_path,
                config[args.dataset_name]["imtopics"] + config[args.dataset_name]["odomtopics"],
            )
        except Exception as e:
            print(e)
            print(f"Error loading {bag_path}. Skipping...")
            continue

        traj_name = "_".join(bag_path.rstrip("/").split("/")[-2:])

        # Pass the extracted data to the processing function
        bag_img_data, bag_traj_data = get_images_and_odom_ros2(
            bag_data,
            config[args.dataset_name]["imtopics"],
            config[args.dataset_name]["odomtopics"],
            eval(config[args.dataset_name]["img_process_func"]),
            eval(config[args.dataset_name]["odom_process_func"]),
            rate=args.sample_rate,
            ang_offset=config[args.dataset_name]["ang_offset"],
        )

        if bag_img_data is None or bag_traj_data is None:
            print(f"{bag_path} did not have the topics we were looking for. Skipping...")
            continue

        cut_trajs = filter_backwards(bag_img_data, bag_traj_data)

        for i, (img_data_i, traj_data_i) in enumerate(cut_trajs):
            traj_name_i = traj_name + f"_{i}"
            traj_folder_i = os.path.join(args.output_dir, traj_name_i)
            os.makedirs(traj_folder_i, exist_ok=True)

            with open(os.path.join(traj_folder_i, "traj_data.pkl"), "wb") as f:
                pickle.dump(traj_data_i, f)

            for i, img in enumerate(img_data_i):
                img.save(os.path.join(traj_folder_i, f"{i}.jpg"))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-name", "-d", default="edubot_1", type=str)
    parser.add_argument("--input-dir", "-i", type=str, default="../rosbags/edubot_1/")
    parser.add_argument("--output-dir", "-o", default="../train_data/edubot_1/", type=str)
    parser.add_argument("--num-trajs", "-n", default=-1, type=int)
    parser.add_argument("--sample-rate", "-s", default=4.0, type=float)

    args = parser.parse_args()
    print(f"STARTING PROCESSING {args.dataset_name.upper()} DATASET")
    main(args)
    print(f"FINISHED PROCESSING {args.dataset_name.upper()} DATASET")
