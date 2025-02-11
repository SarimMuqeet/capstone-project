import fiftyone as fo
import fiftyone.zoo as foz
import os
import csv
import traceback
import logging
import requests
from ultralytics import YOLO  # Ensure you have ultralytics installed: pip install ultralytics
import time
from concurrent.futures import ThreadPoolExecutor, as_completed

# Enable detailed logging but reduce verbosity for performance
logging.basicConfig(level=logging.WARNING)

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

def run_inference_batch(dataset, model, label_field="predictions", batch_size=32):
    """
    Runs inference on the dataset using the provided model and stores predictions in batches.

    Args:
        dataset (fo.Dataset): The FiftyOne dataset to run inference on.
        model: The loaded YOLO model to use for inference.
        label_field (str): The field name to store predictions.
        batch_size (int): Number of images to process in a batch.
    """
    samples = dataset.select_fields(["filepath", "metadata"]).clone()

    # Ensure all samples have metadata
    samples.compute_metadata()

    filepaths = [sample.filepath for sample in samples]
    total = len(filepaths)
    print(f"Running inference on {total} samples with batch size {batch_size}.")

    start_time = time.time()

    for i in range(0, total, batch_size):
        batch_filepaths = filepaths[i:i + batch_size]
        try:
            results = model.predict(source=batch_filepaths, save=False, verbose=False)

            for sample, result in zip(samples[i:i + batch_size], results):
                detections = []
                for pred in result.boxes.data.tolist():
                    x1, y1, x2, y2, conf, cls = pred
                    label = model.names[int(cls)]
                    # Convert to relative coordinates [x, y, width, height]
                    relative_bbox = [
                        x1 / sample.metadata.width,
                        y1 / sample.metadata.height,
                        (x2 - x1) / sample.metadata.width,
                        (y2 - y1) / sample.metadata.height,
                    ]
                    detections.append(
                        fo.Detection(
                            label=label,
                            bounding_box=relative_bbox,
                            confidence=conf,
                        )
                    )
                # Create a Detections object
                detections = fo.Detections(detections=detections)
                # Assign to the sample
                sample[label_field] = detections
            # Periodically save the dataset to avoid memory issues
            if (i // batch_size) % 10 == 0:
                dataset.save()

            elapsed = time.time() - start_time
            print(f"Processed {min(i + batch_size, total)}/{total} samples. Time elapsed: {elapsed:.2f}s")
        except Exception as e:
            print(f"Failed to run inference on batch starting at index {i}: {e}")
            traceback.print_exc()

    # Final save
    dataset.save()
    total_time = time.time() - start_time
    print(f"Inference completed in {total_time / 60:.2f} minutes.")

def evaluate_model(dataset, gt_field="ground_truth", pred_field="predictions"):
    if not dataset.has_field(gt_field):
        print(f"Ground truth field '{gt_field}' does not exist in the dataset '{dataset.name}'. Skipping evaluation.")
        return

    results = dataset.evaluate_detections(
        gt_field=gt_field,
        pred_field=pred_field,
        eval_key="eval",
        iou_threshold=0.5,
        recall_at_iou=[0.5, 0.75],
    )
    print(f"\nEvaluation results for dataset '{dataset.name}':")
    print(results)

    # Correctly accessing metrics
    metrics = results.metrics
    print(f"Precision: {metrics.precision}")
    print(f"Recall: {metrics.recall}")
    print(f"mAP (mean Average Precision): {metrics.mean_average_precision}")
    print(f"mAP@0.5: {metrics.mean_average_precision_at_iou_threshold(0.5)}")
    print(f"mAP@0.75: {metrics.mean_average_precision_at_iou_threshold(0.75)}")

    # Extract per-class metrics
    per_class_metrics = results.mAP_per_class()
    print("\nPer-Class mAP:")
    for cls, mAP in per_class_metrics.items():
        print(f" - {cls}: mAP@0.5 = {mAP['ap50']:.4f}, mAP@0.5:0.95 = {mAP['ap50:95']:.4f}")

def main():
    try:
        start_total_time = time.time()

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

        # Names for the datasets (only validation and test)
        final_datasets = {
            "validation": "open-images-val",
            "test": "open-images-test"
        }

        # Load existing datasets (only validation and test)
        datasets = {}
        for split in ["validation", "test"]:
            dataset_name = final_datasets[split]
            if fo.dataset_exists(dataset_name):
                datasets[split] = fo.load_dataset(dataset_name)
                print(f"Loaded existing dataset '{dataset_name}' with {datasets[split].count()} samples.")
            else:
                print(f"Dataset '{dataset_name}' does not exist. Please ensure it is downloaded. Exiting.")
                return

        # Compute metadata for all datasets (only validation and test)
        for split in ["validation", "test"]:
            dataset = datasets[split]
            # Check if any sample lacks metadata
            sample_without_metadata = dataset.match({"metadata": None}).count()
            if sample_without_metadata > 0:
                print(f"Computing metadata for dataset '{dataset.name}' ({sample_without_metadata} samples missing metadata)...")
                dataset.compute_metadata()
                print(f"Metadata computed for dataset '{dataset.name}'.")
            else:
                print(f"All samples in dataset '{dataset.name}' already have metadata.")

        # Load the YOLOv8n model (adjust the model name/path as needed)
        model = YOLO("yolo11n.pt")  # Ensure the correct model path/name
        print("Loaded YOLO11n model.")

        # Run inference on each split (only validation and test)
        for split in ["validation", "test"]:
            print(f"\nRunning inference on '{split}' split.")
            dataset = datasets[split]
            run_inference_batch(dataset, model, label_field="predictions", batch_size=100)
            print(f"Completed inference on '{split}' split.")

        # Evaluate the model on each split (only validation and test)
        for split in ["validation", "test"]:
            print(f"\nEvaluating model on '{split}' split.")
            dataset = datasets[split]
            # Ensure that ground truth labels are present in 'ground_truth' field
            evaluate_model(dataset, gt_field="ground_truth", pred_field="predictions")

        total_execution_time = time.time() - start_total_time
        print(f"\nTotal script execution time: {total_execution_time / 60:.2f} minutes.")

        # Ensure the script runs within half an hour
        if total_execution_time > 1800:
            print("WARNING: The script took longer than 30 minutes to execute.")

        # Optionally, visualize the results using FiftyOne App
        # session = fo.launch_app(datasets["validation"])
        # session.wait()

    except Exception as e:
        print(f"An unhandled exception occurred: {e}")
        traceback.print_exc()

if __name__ == "__main__":
    main()
