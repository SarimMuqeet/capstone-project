from flask import Flask, render_template, request, jsonify, send_file
import yaml
from PIL import Image
import os

app = Flask(__name__)

# Load map metadata from YAML
with open('map.yaml', 'r') as f:
    map_data = yaml.safe_load(f)
    resolution = map_data['resolution']
    origin = map_data['origin']  # [x_origin, y_origin, theta]
    pgm_path = map_data['image']

# Load PGM image dimensions
pgm_image = Image.open(pgm_path)
width, height = pgm_image.size

# Convert PGM to PNG for web display
png_path = 'converted_map.png'
pgm_image.save(png_path)

# Load the image for pixel access
image_for_pixels = Image.open(pgm_path).convert('L')  # Convert to grayscale
pixel_map = image_for_pixels.load()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/map.png')
def serve_map():
    return send_file(png_path)

@app.route('/convert', methods=['POST'])
def convert_coordinates():
    data = request.get_json()
    pixel_x = data['x']
    pixel_y = data['y']

    pixel_value = pixel_map[pixel_x, pixel_y]
    if pixel_value == 0:  # 0 = black in 8-bit PGM
        return jsonify({'error': 'Cannot select obstacles (black pixels)'}), 400

    # Calculate real-world coordinates
    real_y = origin[0] + ((width-pixel_x) * resolution)
    real_x = origin[1] + ((height - pixel_y) * resolution)  # Flip y-axis

    return jsonify({
        'x': round(real_x, 3),
        'y': round(real_y, 3),
        'z': 0.0  # Assume 2D map
    })

if __name__ == '__main__':
    app.run(debug=True)
