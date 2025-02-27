import os
import shutil

def flatten_directory_structure(base_dir):
    for split in ['train', 'validation', 'test']:
        image_dir = os.path.join(base_dir, 'images', split)
        label_dir = os.path.join(base_dir, 'labels', split)
        yolo_label_dir = os.path.join(base_dir, 'yoloLabels', split)  # Assuming YOLO labels are stored here

        # Move images from subdirectories to the main directory
        for root, _, files in os.walk(image_dir):
            if root != image_dir:
                for file in files:
                    src = os.path.join(root, file)
                    dst = os.path.join(image_dir, file)
                    shutil.move(src, dst)

        # Move labels from subdirectories to the main directory
        for root, _, files in os.walk(label_dir):
            if root != label_dir:
                for file in files:
                    src = os.path.join(root, file)
                    dst = os.path.join(label_dir, file)
                    shutil.move(src, dst)

        # Move YOLO labels from subdirectories to the main directory
        for root, _, files in os.walk(yolo_label_dir):
            if root != yolo_label_dir:
                for file in files:
                    src = os.path.join(root, file)
                    dst = os.path.join(yolo_label_dir, file)
                    shutil.move(src, dst)

        # Remove empty subdirectories
        for root, dirs, _ in os.walk(image_dir, topdown=False):
            for d in dirs:
                try:
                    os.rmdir(os.path.join(root, d))
                except OSError:
                    pass  # Skip non-empty directories

        for root, dirs, _ in os.walk(label_dir, topdown=False):
            for d in dirs:
                try:
                    os.rmdir(os.path.join(root, d))
                except OSError:
                    pass  # Skip non-empty directories

        for root, dirs, _ in os.walk(yolo_label_dir, topdown=False):
            for d in dirs:
                try:
                    os.rmdir(os.path.join(root, d))
                except OSError:
                    pass  # Skip non-empty directories

# Example usage
base_directory = './dataset'
flatten_directory_structure(base_directory)
