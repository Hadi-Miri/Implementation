# **Fixing Marker Movement Logic in Python (Qt)**
This document explains the difference between the **Java (Android)** and **Python (Qt)** implementations of the marker movement logic for `/ego_pose`, highlights why the Python version didn't work, and provides the necessary fixes.

---

## **1. Understanding the Issue**
### **What Works in Java (Android)**
- Subscribes to `/ego_pose` via WebSocket.
- Extracts `x, y` from the message.
- Scales and offsets the values before moving the marker:
  ```java
  pointerX = (((y / 100.0) * mapWidth) * 12) + 50;
  pointerY = (((x / 100.0) * mapHeight) * 12) + 50;

    Uses ImageView for the marker.

What Fails in Python (Qt)

    Incorrect coordinate mapping:
        Java swaps x and y (y -> X-axis, x -> Y-axis), but Python didn't.
    No offset added in Python (+50 in Java).
    Scaling was different (Java dynamically scales based on mapWidth, Python assumed fixed 800x800).

2. Fixing Python Implementation
🚀 Fix 1: Adjust Scaling and Axis Swap

Modify transform_coordinates() to:

    Swap x and y.
    Apply scaling factors like Java.
    Add an offset of +50.

def transform_coordinates(self, real_x, real_y):
    """
    Converts real-world coordinates to pixel coordinates.
    Swaps x and y to match Java's logic.
    """
    map_width = self.map_container.width()   # Get dynamic width
    map_height = self.map_container.height() # Get dynamic height

    # Match Java's scaling logic
    scale_factor_x = map_width / 100.0 * 12
    scale_factor_y = map_height / 100.0 * 12

    # Swap axes & add offset
    pixel_x = (real_y * scale_factor_x) + 50
    pixel_y = (real_x * scale_factor_y) + 50

    return int(pixel_x), int(pixel_y)

🚀 Fix 2: Ensure the Marker Moves

Modify update_position() to:

    Call the corrected transform_coordinates().
    Print debug values to check correctness.

@QtCore.pyqtSlot(float, float)
def update_position(self, real_x, real_y):
    """
    Update marker position using real coordinates transformed into UI coordinates.
    """
    if not self.isVisible():
        return

    pixel_x, pixel_y = self.transform_coordinates(real_x, real_y)
    self.marker_label.move(pixel_x, pixel_y)

    # Debugging
    print(f"Updated marker position: Real({real_x}, {real_y}) -> Pixel({pixel_x}, {pixel_y})")

    self.marker_label.setText(f"🚗 ({pixel_x}, {pixel_y})")
    self.marker_label.adjustSize()

3. Summary of Fixes

✅ Swapped X & Y mapping to match Java.
✅ Adjusted scaling using map_width / 100.0 * 12 to match Java.
✅ Added offset +50 like Java.
✅ Added debug print statements to check correctness.
4. Final Steps

    Test the Fix: Run the Python UI and check if the marker updates correctly.
    If Still Incorrect:
        Print real_x, real_y and pixel_x, pixel_y to compare with Java’s values.
    If Scaling is Off:
        Ensure map_width and map_height are correctly set.
