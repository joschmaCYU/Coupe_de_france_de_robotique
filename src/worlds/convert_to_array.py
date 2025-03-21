from PIL import Image
import numpy as np
import json

def image_to_array(image_path, color_map):
    """
    Converts an image into a 2D array based on a color-to-number mapping.
    :param image_path: Path to the input image.
    :param color_map: Dictionary mapping (R, G, B) tuples to numbers.
    :return: 2D numpy array.
    """
    image = Image.open(image_path).convert('RGB')  # Ensure image is in RGB mode
    pixels = np.array(image)
    
    height, width, _ = pixels.shape
    output_array = np.zeros((height, width), dtype=int)
    
    for y in range(height):
        for x in range(width):
            color = tuple(pixels[y, x])  # Get (R, G, B) tuple
            output_array[y, x] = color_map.get(color, -1)  # Default to -1 if color not found
    
    return output_array

def save_array_to_file(array, output_path):
    """Saves the 2D array to a file in JSON format."""
    with open(output_path, 'w') as f:
        json.dump(array.tolist(), f)

if __name__ == "__main__":
    color_mapping = {
        (255, 255, 255): 1,     # White (sol) -> 01
        (127, 127, 127): 10,    # Grey (stockage) -> 10

        (255, 255, 0): 31,      # Yellow (construction) -> 31
        (0, 0, 255): 32,        # Red (construction) -> 32

        (255, 127, 127): 41,    # Yellow/Rouge/Blue (Stock de matière premère réservé) -> 41
        (0, 127, 255): 42,      # Blue/Green (Stock de matière premère réservé) -> 42

        (255, 255, 127): 51,    # Yellow (Départ/Arrivé) -> 51
        (127, 127, 255): 52     # Blue (Départ/Arrivé) -> 52
    }
    
    image_path = "/home/josch/ROS2/src/robot_creation/src/worlds/Map_CDFDR.png"  # Change to your image path
    output_path = "/home/josch/ROS2/src/robot_creation/src/worlds/output.json"  # Output file path
    
    array_result = image_to_array(image_path, color_mapping)
    save_array_to_file(array_result, output_path)
    print(f"Array saved to {output_path}")

