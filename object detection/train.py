import fiftyone as fo
import os


train_dataset = fo.load_dataset("open-images-train")
val_dataset = fo.load_dataset("open-images-val")
test_dataset = fo.load_dataset("open-images-test")

classes = [
    "Book",
    "Clothing",
    "Coffee cup",
    "Toy",
    "Bottle",
    "Footwear",
    "Pillow",
    "Houseplant",
    "Lamp",
    "Box"
]

export_dir = "C:/Users/natha/Documents/Homey Helper/object detection/datasets"

os.makedirs(os.path.join(export_dir, "train"), exist_ok=True)
os.makedirs(os.path.join(export_dir, "val"), exist_ok=True)
os.makedirs(os.path.join(export_dir, "test"), exist_ok=True)

train_dataset.export(
    export_dir=os.path.join(export_dir, "train"),
    dataset_type=fo.types.YOLOv5Dataset,
    label_field="ground_truth",
    classes=classes,
)

val_dataset.export(
    export_dir=os.path.join(export_dir, "val"),
    dataset_type=fo.types.YOLOv5Dataset,
    label_field="ground_truth",
    classes=classes,
)

test_dataset.export(
    export_dir=os.path.join(export_dir, "test"),
    dataset_type=fo.types.YOLOv5Dataset,
    label_field="ground_truth",
    classes=classes,
)

