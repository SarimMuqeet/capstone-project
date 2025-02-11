import os
import pandas as pd
from PIL import Image

# Configuration
annotations_file = './dataSets/boxes/oidv6-train-annotations-bbox.csv'
images_dir = './dataSets/images'
output_dir = './dataSets/output'

# Load annotations
annotations = pd.read_csv(annotations_file)

# Define class ID mapping
label_to_class_id = {
    '/m/0bt_c3': 0,  # book
    '/m/04dr76w': 1, # bottle
    '/m/01x3z': 2, # clock
    '/m/09j2d': 3, # clothing
    '/m/02p5f1q': 4, #coffee_cup
    '/m/03fp41': 5, #houseplant
    '/m/06z37_': 6, #picture frame
    '/m/034c16': 7, #pillow
    '/m/0138tl': 8, #toy
    '/m/050k8': 9, #mobile_phone
}

def convert_to_yolo_format(row, img_width, img_height):
    # Convert bounding box coordinates from [x_min, y_min, x_max, y_max] to YOLO format
    x_min, y_min, x_max, y_max = row['XMin'], row['YMin'], row['XMax'], row['YMax']
    x_center = (x_min + x_max) / 2
    y_center = (y_min + y_max) / 2
    width = x_max - x_min
    height = y_max - y_min
    
    # Normalize values to [0, 1] by dividing by image dimensions
    x_center /= img_width
    y_center /= img_height
    width /= img_width
    height /= img_height
    
    return f"{x_center} {y_center} {width} {height}"

def process_images(split):
    split_dir = os.path.join(images_dir, split)
    for class_name in os.listdir(split_dir):
        class_dir = os.path.join(split_dir, class_name)
        if not os.path.isdir(class_dir):
            continue
        
        class_output_dir = os.path.join(output_dir, split, class_name)
        os.makedirs(class_output_dir, exist_ok=True)

        for image_file in os.listdir(class_dir):
            if not image_file.endswith('.jpg'):
                continue
            
            image_id = os.path.splitext(image_file)[0]
            image_path = os.path.join(class_dir, image_file)
            yolo_file_path = os.path.join(class_output_dir, image_id + '.txt')

            print(f"Processing image: {image_path}")
            print(f"Output annotation file: {yolo_file_path}")

            if not os.path.exists(image_path):
                print(f"Image {image_path} not found.")
                continue

            try:
                with Image.open(image_path) as img:
                    img_width, img_height = img.size

                # Filter annotations for the current image
                image_annotations = annotations[annotations['ImageID'] == image_id]

                with open(yolo_file_path, 'w') as f:
                    for _, row in image_annotations.iterrows():
                        class_id = label_to_class_id.get(row['LabelName'])
                        if class_id is not None:
                            yolo_format = convert_to_yolo_format(row, img_width, img_height)
                            f.write(f"{class_id} {yolo_format}\n")
                        else:
                            print(f"Label {row['LabelName']} not found in YOLO mappings.")
            except IOError as e:
                print(f"Error processing image {image_path}: {e}")

# Process train, validation, and test splits
for split in ['train', 'validation', 'test']:
    process_images(split)

print("Conversion complete.")


# import os
# from PIL import Image

# # Configuration
# images_dir = './dataSets/images'
# output_dir = './dataSets/output'
# os.makedirs(output_dir, exist_ok=True)

# def process_images(split):
#     split_dir = os.path.join(images_dir, split)
#     for class_name in os.listdir(split_dir):
#         class_dir = os.path.join(split_dir, class_name)
#         if not os.path.isdir(class_dir):
#             continue
        
#         class_output_dir = os.path.join(output_dir, split, class_name)
#         os.makedirs(class_output_dir, exist_ok=True)

#         for image_file in os.listdir(class_dir):
#             if not image_file.endswith('.jpg'):
#                 continue
            
#             image_id = os.path.splitext(image_file)[0]
#             image_path = os.path.join(class_dir, image_file)
#             yolo_file_path = os.path.join(class_output_dir, image_id + '.txt')

#             print(f"Processing image: {image_path}")
#             print(f"Output annotation file: {yolo_file_path}")

#             if not os.path.exists(image_path):
#                 print(f"Image {image_path} not found.")
#                 continue

#             try:
#                 with Image.open(image_path) as img:
#                     img_width, img_height = img.size
#                     # Example conversion logic (replace with actual logic)
#                     with open(yolo_file_path, 'w') as f:
#                         f.write("0 0.5 0.5 0.2 0.3\n")
#             except IOError as e:
#                 print(f"Error processing image {image_path}: {e}")

# # Process train, validation, and test splits
# for split in ['train', 'validation', 'test']:
#     process_images(split)

# print("Conversion complete.")


# import pandas as pd
# import os
# from PIL import Image

# # Configuration
# annotations_file = './dataSets/boxes/oidv6-train-annotations-bbox.csv'
# images_dir = './dataSets/images'
# output_dir = './dataSets/output'
# os.makedirs(output_dir, exist_ok=True)

# # Load annotations
# annotations = pd.read_csv(annotations_file)

# unique_labels = annotations['LabelName'].unique()

# valid_classes = set(['/m/0bt_c3', '/m/04dr76w', '/m/01x3z', '/m/09j2d', '/m/02p5f1q', '/m/03fp41', '/m/06z37_', '/m/034c16', '/m/0138tl', '/m/050k8'])
# filtered_annotations = annotations[annotations['LabelName'].isin(valid_classes)]

# # Mapping from Open Images class IDs to YOLO class IDs
# label_to_class_id = {
#     '/m/0bt_c3': 0,  # book
#     '/m/04dr76w': 1, # bottle
#     '/m/01x3z': 2, # clock
#     '/m/09j2d': 3, # clothing
#     '/m/02p5f1q': 4, #coffee_cup
#     '/m/03fp41': 5, #houseplant
#     '/m/06z37_': 6, #picture frame
#     '/m/034c16': 7, #pillow
#     '/m/0138tl': 8, #toy
#     '/m/050k8': 9, #mobile_phone
# }

# def convert_to_yolo_format(row, img_width, img_height):
#     x_min = row['XMin'] * img_width
#     x_max = row['XMax'] * img_width
#     y_min = row['YMin'] * img_height
#     y_max = row['YMax'] * img_height

#     x_center = ((x_min + x_max) / 2.0) / img_width
#     y_center = ((y_min + y_max) / 2.0) / img_height
#     width = (x_max - x_min) / img_width
#     height = (y_max - y_min) / img_height

#     return f"{x_center} {y_center} {width} {height}"

# # Iterate over each image and its annotations
# for image_id, group in filtered_annotations.groupby('ImageID'):
#     image_path = os.path.join(images_dir, image_id + '.jpg')
    
#     if not os.path.exists(image_path):
#         print(f"Image {image_id}.jpg not found.")
#         continue
    
#     # Get image dimensions
#     with Image.open(image_path) as img:
#         img_width, img_height = img.size

#     # Create YOLO annotation file
#     yolo_file_path = os.path.join(output_dir, image_id + '.txt')
#     with open(yolo_file_path, 'w') as f:
#         for _, row in group.iterrows():
#             class_id = label_to_class_id.get(row['LabelName'])
#             if class_id is not None:
#                 yolo_format = convert_to_yolo_format(row, img_width, img_height)
#                 f.write(f"{class_id} {yolo_format}\n")

# print("Conversion complete.")
