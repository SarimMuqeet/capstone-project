import os

def rename_files_in_folder(folder_path, prefix):
    # Ensure the folder exists
    if not os.path.exists(folder_path):
        print(f"The folder {folder_path} does not exist.")
        return
    
    # Get list of files in the folder
    files = os.listdir(folder_path)
    
    # Initialize the index
    index = 0
    
    for filename in files:
        # Get the file extension
        file_extension = os.path.splitext(filename)[1]
        
        # Create the new name
        new_name = f"{prefix}_{index}{file_extension}"
        
        # Create full path for the original and new file names
        original_file = os.path.join(folder_path, filename)
        new_file = os.path.join(folder_path, new_name)
        
        # Rename the file
        os.rename(original_file, new_file)
        print(f"Renamed: {filename} -> {new_name}")
        
        # Increment the index
        index += 1

if __name__ == "__main__":
    # Define the folder path and prefix
    folder_path = input("Enter the folder path: ")
    prefix = input("Enter the prefix for the new filenames: ")
    
    # Rename the files
    rename_files_in_folder(folder_path, prefix)
