import fiftyone as fo
import fiftyone.zoo as foz
import os
import csv
import traceback
import logging
import requests

# Enable detailed logging
logging.basicConfig(level=logging.INFO)

def get_exact_class_names(target_classes, classes_csv_path):
    """
    Retrieves the exact class names from the class descriptions CSV.

    Args:
        target_classes (list): List of human-readable class names to search for.
        classes_csv_path (str): Path to 'classes.csv'.

    Returns:
        list: List of exact class names found.
    """
    exact_classes = []
    with open(classes_csv_path, 'r', encoding='utf-8') as f:
        reader = csv.reader(f)
        class_dict = {row[1].lower(): row[1] for row in reader}
        for target_class in target_classes:
            exact_class = class_dict.get(target_class.lower())
            if exact_class:
                exact_classes.append(exact_class)
            else:
                print(f"WARNING: Class '{target_class}' not found in class descriptions. It will be skipped.")
    return exact_classes

def download_class_descriptions_csv(classes_csv_path):
    """
    Downloads the class descriptions CSV file if it doesn't exist.

    Args:
        classes_csv_path (str): Path to save 'classes.csv'.

    Returns:
        bool: True if the file was downloaded successfully, False otherwise.
    """
    url = "https://storage.googleapis.com/openimages/v6/oidv6-class-descriptions.csv"
    response = requests.get(url)
    if response.status_code == 200:
        os.makedirs(os.path.dirname(classes_csv_path), exist_ok=True)
        with open(classes_csv_path, 'wb') as f:
            f.write(response.content)
        print(f"Class descriptions file downloaded to '{classes_csv_path}'.")
    else:
        print(f"Failed to download class descriptions file from {url}. Status code: {response.status_code}")
        return False
    return True

def main():
    try:
        # Delete existing datasets if you want to start fresh
        for dataset_name in ["open-images-train", "open-images-val", "open-images-test"]:
            if fo.dataset_exists(dataset_name):
                fo.delete_dataset(dataset_name)
                print(f"Deleted existing dataset '{dataset_name}'")

        # Path to class descriptions CSV
        classes_csv_path = os.path.join(fo.config.default_dataset_dir, "open-images-v7", "metadata", "classes.csv")

        # Verify that the class descriptions file exists
        if not os.path.exists(classes_csv_path):
            print(f"Class descriptions file not found at '{classes_csv_path}'. Downloading it...")
            success = download_class_descriptions_csv(classes_csv_path)
            if not success:
                print("Failed to download the class descriptions file. Exiting.")
                return

        # Define your selected classes
        selected_classes_input = [
            "Book",
            "Clothing",
            "Coffee Cup",
            "Toy",
            "Bottle",
            "Footwear",
            "Pillow",
            "Houseplant",
            "Lamp",
            "Box"
        ]

        # Convert to exact class names
        selected_classes = get_exact_class_names(selected_classes_input, classes_csv_path)
        for cls_input, cls_exact in zip(selected_classes_input, selected_classes):
            print(f"Selected class: '{cls_input}' as '{cls_exact}'")

        if not selected_classes:
            print("ERROR: No valid classes found. Exiting.")
            return

        # Define the desired number of samples per split
        split_counts = {
            "train": 100000,
            "validation": 10000,
            "test": 10000
        }

        # Names for the final datasets
        final_datasets = {
            "train": "open-images-train",
            "validation": "open-images-val",
            "test": "open-images-test"
        }

        # Create or load datasets for splits
        datasets = {}
        for split in ["train", "validation", "test"]:
            if fo.dataset_exists(final_datasets[split]):
                datasets[split] = fo.load_dataset(final_datasets[split])
                print(f"Loaded existing dataset '{final_datasets[split]}' with {datasets[split].count()} samples.")
            else:
                datasets[split] = fo.Dataset(final_datasets[split])
                datasets[split].persistent = True
                print(f"Created new dataset '{final_datasets[split]}'.")

        # Sampling Process for each split
        for split in ["train", "validation", "test"]:
            print(f"\nProcessing split: {split}")
            total_needed = split_counts[split]

            try:
                # Load the dataset for the split
                dataset = foz.load_zoo_dataset(
                    "open-images-v7",
                    split=split,
                    label_types=["detections"],
                    classes=selected_classes,
                    max_samples=total_needed,
                    shuffle=True,
                    dataset_name=None,  # Do not persist the temporary dataset
                    download_if_necessary=True,
                    drop_existing_dataset=True,
                    seed=51,
                )
            except Exception as e:
                print(f"Failed to load dataset for split '{split}': {e}")
                traceback.print_exc()
                continue

            available_samples = dataset.count()
            print(f"  Total available samples in '{split}' split: {available_samples}")

            if available_samples == 0:
                print(f"  WARNING: No samples available in split '{split}'. Skipping.")
                continue

            if available_samples < total_needed:
                print(f"Warning: Not enough samples in '{split}' split ({available_samples} available, {total_needed} needed)")
                total_needed = available_samples

            # Take the required number of samples
            dataset_view = dataset.take(total_needed)

            # Add samples to the dataset
            datasets[split].add_samples(dataset_view)

            print(f"  Added {dataset_view.count()} samples to '{split}' split.")

        # Save datasets
        for split in ["train", "validation", "test"]:
            datasets[split].persistent = True

        # Summary
        print("\nProcessing completed. Summary:")
        for split in ["train", "validation", "test"]:
            print(f"  Dataset '{final_datasets[split]}' now has {datasets[split].count()} samples.")

    except Exception as e:
        print(f"An unhandled exception occurred: {e}")
        traceback.print_exc()

if __name__ == "__main__":
    main()
