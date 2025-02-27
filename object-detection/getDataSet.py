import os
import sys
import pandas as pd
import subprocess
import cv2
from tqdm import tqdm

BASE_DIR = "OIDv6"
CSV_DIR = os.path.join(BASE_DIR, "csv_folder")
DATASET_DIR = os.path.join(BASE_DIR, "Dataset")
LABELS_DIR = os.path.join(BASE_DIR, "labels")

def setup_directories():
    os.makedirs("OIDv6/csv_folder", exist_ok=True)  # For metadata
    os.makedirs("OIDv6/Dataset", exist_ok=True)     # For images
    for split in ['train', 'validation', 'test']:
        os.makedirs(f"OIDv6/Dataset/{split}", exist_ok=True)

def run_command(command):
    """Execute shell command with error handling"""
    try:
        result = subprocess.run(command, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        print(result.stdout)
        return True
    except subprocess.CalledProcessError as e:
        print(f"Command failed: {' '.join(e.cmd)}\nError:\n{e.stderr}")
        return False

def download_annotations():
    print("Downloading class descriptions...")
    try:
        subprocess.run([
            "oidv6", "downloader",
            "--dataset", "OIDv6",  # Use default directory
            "--type_data", "all",
            "--classes", "Coffee cup", "Book", "Bottle", "Clock", "Clothing", 
                      "Houseplant", "Picture frame", "Pillow", "Toy", "Mobile phone",
            "--yes",
            "--download_missing"
        ], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error downloading annotations: {e}")
        sys.exit(1)

def download_data(split, class_list, limit):
    """Download dataset split with error handling"""
    print(f"\nDownloading {split} data...")
    success = run_command([
        "openimages", "download", "--dataset", DATASET_DIR, "--type", split,
        "--classes", ",".join(class_list), "--max", str(limit), "--download_missing"
    ])
    if not success:
        raise RuntimeError(f"Failed to download {split} data")

def get_class_info(classes):
    """Retrieve class information with path validation"""
    class_path = os.path.join(CSV_DIR, "class-descriptions-boxable.csv")
    if not os.path.exists(class_path):
        raise FileNotFoundError(f"Class descriptions not found at {class_path}")

    return pd.read_csv(class_path, header=None, names=['LabelName', 'DisplayName']).query("DisplayName in @classes")

def convert_to_yolo_format(classes):
    """Convert annotations to YOLO format with improved error handling"""
    class_path = os.path.join(CSV_DIR, "class-descriptions-boxable.csv")
    if not os.path.exists(class_path):
        print("Skipping YOLO conversion - class descriptions missing")
        return

    class_df = pd.read_csv(class_path, header=None, names=['LabelName', 'DisplayName'])
    class_map = {row['LabelName']: idx for idx, row in class_df.iterrows()}

    for split in ['train', 'validation', 'test']:
        ann_path = os.path.join(CSV_DIR, f"{split}-annotations-bbox.csv")
        if not os.path.exists(ann_path):
            print(f"Skipping {split} - annotations file not found")
            continue

        annotations = pd.read_csv(ann_path, low_memory=False)
        for img_id, group in tqdm(annotations.groupby('ImageID'), desc=f"Processing {split}"):
            img_path = os.path.join(DATASET_DIR, split, f"{img_id}.jpg")
            label_path = os.path.join(LABELS_DIR, split, f"{img_id}.txt")

            if not os.path.exists(img_path):
                continue

            try:
                img = cv2.imread(img_path)
                if img is None:
                    os.remove(img_path)
                    continue

                h, w = img.shape[:2]
                with open(label_path, 'w') as f:
                    for _, row in group.iterrows():
                        if row['LabelName'] not in class_map:
                            continue
                        x_center = (row['XMin'] + row['XMax']) / 2
                        y_center = (row['YMin'] + row['YMax']) / 2
                        width = row['XMax'] - row['XMin']
                        height = row['YMax'] - row['YMin']
                        
                        f.write(f"{class_map[row['LabelName']]} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}\n")
            except Exception as e:
                print(f"Error processing {img_id}: {str(e)}")
                if os.path.exists(img_path):
                    os.remove(img_path)

def main():
    try:
        setup_directories()
        download_annotations()

        classes = ['Coffee cup', 'Book', 'Bottle', 'Clock', 'Clothing',
                   'Houseplant', 'Picture frame', 'Pillow', 'Toy', 'Mobile phone']
        limits = {'train': 25000, 'validation': 2500, 'test': 2500}

        class_info = get_class_info(classes)
        verified_classes = class_info['DisplayName'].tolist()

        print(f"Verified classes: {verified_classes}")

        for split in ['train', 'validation', 'test']:
            download_data(split, verified_classes, limits[split])

        convert_to_yolo_format(classes)
        print("Dataset preparation completed successfully!")

    except Exception as e:
        print(f"Fatal error: {str(e)}")
        sys.exit(1)

if __name__ == "__main__":
    main()
