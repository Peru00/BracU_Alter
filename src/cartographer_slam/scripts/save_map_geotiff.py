#!/usr/bin/env python3
"""
Save Cartographer occupancy grid map as GeoTIFF.

This script subscribes to the /map topic (OccupancyGrid) and saves it as a GeoTIFF file.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import argparse
import sys

try:
    from osgeo import gdal, osr
    HAS_GDAL = True
except ImportError:
    HAS_GDAL = False
    print("WARNING: GDAL not found. Will use alternative method with rasterio.")
    try:
        import rasterio
        from rasterio.transform import from_bounds
        HAS_RASTERIO = True
    except ImportError:
        HAS_RASTERIO = False


class MapSaverGeoTiff(Node):
    """Node to save occupancy grid as GeoTIFF."""

    def __init__(self, output_path: str, map_topic: str = '/map'):
        super().__init__('map_saver_geotiff')
        
        self.output_path = output_path
        self.map_received = False
        
        self.get_logger().info(f'Subscribing to map topic: {map_topic}')
        self.get_logger().info(f'Output will be saved to: {output_path}')
        
        self.subscription = self.create_subscription(
            OccupancyGrid,
            map_topic,
            self.map_callback,
            10
        )

    def map_callback(self, msg: OccupancyGrid):
        """Handle incoming occupancy grid message."""
        if self.map_received:
            return
            
        self.get_logger().info(
            f'Received map: {msg.info.width}x{msg.info.height}, '
            f'resolution: {msg.info.resolution}m/pixel'
        )
        
        # Convert occupancy grid to numpy array
        # OccupancyGrid values: -1 (unknown), 0-100 (free to occupied)
        data = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width)
        )
        
        # Convert to image format (0-255)
        # -1 (unknown) -> 205 (gray)
        # 0 (free) -> 254 (white)
        # 100 (occupied) -> 0 (black)
        image = np.zeros((msg.info.height, msg.info.width), dtype=np.uint8)
        
        # Unknown areas (value -1)
        image[data == -1] = 205
        
        # Free space (value 0)
        image[data == 0] = 254
        
        # Occupied space (value 100)
        image[data == 100] = 0
        
        # Gradient for values between 0-100
        mask = (data > 0) & (data < 100)
        image[mask] = (100 - data[mask]).astype(np.uint8) * 255 // 100
        
        # Flip vertically (image origin is bottom-left, TIFF origin is top-left)
        image = np.flipud(image)
        
        # Get map origin and bounds
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        resolution = msg.info.resolution
        
        width = msg.info.width
        height = msg.info.height
        
        # Calculate bounds
        min_x = origin_x
        max_x = origin_x + width * resolution
        min_y = origin_y
        max_y = origin_y + height * resolution
        
        self.get_logger().info(
            f'Map bounds: ({min_x:.2f}, {min_y:.2f}) to ({max_x:.2f}, {max_y:.2f})'
        )
        
        # Save as GeoTIFF
        success = False
        
        if HAS_GDAL:
            success = self._save_with_gdal(image, min_x, max_x, min_y, max_y, resolution)
        elif HAS_RASTERIO:
            success = self._save_with_rasterio(image, min_x, max_x, min_y, max_y)
        else:
            self.get_logger().error(
                'Neither GDAL nor rasterio is available. '
                'Please install: pip install gdal or pip install rasterio'
            )
            
        if success:
            self.map_received = True
            self.get_logger().info(f'Map saved successfully to: {self.output_path}')
            
            # Also save a simple PNG for quick viewing
            png_path = self.output_path.replace('.tif', '.png').replace('.tiff', '.png')
            if png_path == self.output_path:
                png_path = self.output_path + '.png'
            self._save_png(image, png_path)
        else:
            self.get_logger().error('Failed to save map')

    def _save_with_gdal(self, image: np.ndarray, min_x: float, max_x: float, 
                        min_y: float, max_y: float, resolution: float) -> bool:
        """Save GeoTIFF using GDAL."""
        try:
            driver = gdal.GetDriverByName('GTiff')
            height, width = image.shape
            
            dataset = driver.Create(
                self.output_path, 
                width, height, 1, 
                gdal.GDT_Byte
            )
            
            # Set geotransform: (top_left_x, pixel_width, 0, top_left_y, 0, -pixel_height)
            geotransform = (min_x, resolution, 0, max_y, 0, -resolution)
            dataset.SetGeoTransform(geotransform)
            
            # Set projection (local coordinate system)
            srs = osr.SpatialReference()
            srs.SetLocalCS("ROS Map Coordinate System")
            dataset.SetProjection(srs.ExportToWkt())
            
            # Write data
            band = dataset.GetRasterBand(1)
            band.WriteArray(image)
            band.SetNoDataValue(205)  # Unknown areas
            band.FlushCache()
            
            dataset = None  # Close file
            return True
            
        except Exception as e:
            self.get_logger().error(f'GDAL error: {e}')
            return False

    def _save_with_rasterio(self, image: np.ndarray, min_x: float, max_x: float,
                            min_y: float, max_y: float) -> bool:
        """Save GeoTIFF using rasterio."""
        try:
            height, width = image.shape
            
            transform = from_bounds(min_x, min_y, max_x, max_y, width, height)
            
            with rasterio.open(
                self.output_path,
                'w',
                driver='GTiff',
                height=height,
                width=width,
                count=1,
                dtype=image.dtype,
                crs='+proj=longlat +datum=WGS84 +no_defs',  # Placeholder CRS
                transform=transform,
                nodata=205
            ) as dst:
                dst.write(image, 1)
                
            return True
            
        except Exception as e:
            self.get_logger().error(f'Rasterio error: {e}')
            return False

    def _save_png(self, image: np.ndarray, path: str):
        """Save a simple PNG for quick viewing."""
        try:
            from PIL import Image
            img = Image.fromarray(image)
            img.save(path)
            self.get_logger().info(f'PNG preview saved to: {path}')
        except ImportError:
            self.get_logger().warn('PIL not available, skipping PNG export')
        except Exception as e:
            self.get_logger().warn(f'Could not save PNG: {e}')


def main(args=None):
    parser = argparse.ArgumentParser(description='Save ROS2 map as GeoTIFF')
    parser.add_argument(
        '-o', '--output', 
        type=str, 
        default='map.tif',
        help='Output GeoTIFF file path (default: map.tif)'
    )
    parser.add_argument(
        '-t', '--topic',
        type=str,
        default='/map',
        help='Map topic to subscribe to (default: /map)'
    )
    
    # Parse known args to handle ROS args
    parsed_args, remaining = parser.parse_known_args()
    
    rclpy.init(args=remaining)
    
    node = MapSaverGeoTiff(
        output_path=parsed_args.output,
        map_topic=parsed_args.topic
    )
    
    try:
        # Spin until map is received
        while rclpy.ok() and not node.map_received:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
