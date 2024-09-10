import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

import math
import time

class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__("filtered_pc2_subscriber")
        self.subscription = self.create_subscription(
            PointCloud2,
            '/filtered_points',
            self.listener_callback,
            10
        )
        self.subscription
        self.points = []
        self.init_workload = 0.0
        self.percent_workload = 0.0
        self.init_bool = True


    def listener_callback(self, msg):
        self.points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        self.init_plt()


    def calculate_workload(self, dx, dy, dz):
        workload = 0
        dig_workload = 1.5 * 1.5 * 0.2
        for i in range(len(dx)):
            workload += dx[i] * dy[i] * dz[i]
        return workload + dig_workload


    def dot_2d_plt(self, bin_points, x_offset, y_offset, x_bins, y_bins, i, j, x_split):
        crop_num = x_split + 1
        crop_xmin, crop_xmax = -crop_num, crop_num
        crop_ymin, crop_ymax = -crop_num, crop_num

        transformed_x = np.array([])
        transformed_y = np.array([])
        # Draw 2D dot graph
        for (x, y, _) in bin_points:
            transformed_x = np.append(transformed_x, x - x_offset)
            transformed_y = np.append(transformed_y, y - y_offset)

        # transformed_x = np.array([x - x_offset for (x, _, _) in bin_points])
        # transformed_y = np.array([y - y_offset for (_, y, _) in bin_points])

        crop_mask = (
            (transformed_x >= crop_xmin) & (transformed_x <= crop_xmax) &
            (transformed_y >= crop_ymin) & (transformed_y <= crop_ymax)
        )
        transformed_x = np.array(transformed_x)[crop_mask]
        transformed_y = np.array(transformed_y)[crop_mask]

        if len(transformed_x) > 0 and len(transformed_y) > 0:
            counts, _, _ = np.histogram2d(transformed_x, transformed_y, bins=[x_bins, y_bins])

            plt.scatter(transformed_x, transformed_y, color='blue', marker='o')
            plt.vlines(x_bins, ymin = crop_ymin, ymax = crop_ymax, color = 'gray', linestyle = '--', linewidth = 1.0)
            plt.hlines(y_bins, xmin = crop_xmin, xmax = crop_xmax, color = 'gray', linestyle = '--', linewidth = 1.0)

            if counts[i, j] > 0:
                plt.text(
                    x = (x_bins[i] + x_bins[i + 1]) / 2,
                    y = (y_bins[j] + y_bins[j + 1]) / 2,
                    s = int(counts[i, j]),
                    color = 'red',
                    ha = 'center',
                    va = 'center',
                    fontsize=15,
                )
        plt.xlim(crop_xmin, crop_xmax)
        plt.ylim(crop_xmin, crop_xmax)
        plt.xlabel('X axis')
        plt.ylabel('Y axis')
        return


    def init_plt(self):
        start = time.time()
        current_workload = 0.0
        xmin, xmax, ymin, ymax, zmin, zmax = -2.5, 2.5, -2.5, 2.5, -2, 5
        x_split = y_split = 0.5 # value: 0.5, 0.1, 0.07 ...
        z_split = 0.5

        x_offset = 2.0
        y_offset = -0.0

        # Create bins based on the split values
        x_bins = np.arange(xmin, xmax + x_split, x_split)
        y_bins = np.arange(ymin, ymax + y_split, y_split)

        # Initialize arrays for bar positions and sizes
        x_center = []
        y_center = []
        dz = []
        dx = dy = np.ones((len(x_bins) - 1) * (len(y_bins) - 1)) * x_split

        # Compute average z for each bin
        for i in range(len(x_bins) - 1):
            for j in range(len(y_bins) - 1):
                x_min_bin, x_max_bin = x_bins[i], x_bins[i + 1]
                y_min_bin, y_max_bin = y_bins[j], y_bins[j + 1]

                # Filter points within the current bin
                bin_points = [
                    (x, y, z) for (x, y, z) in self.points
                    if (x_min_bin <= x - x_offset < x_max_bin) and (y_min_bin <= y - y_offset < y_max_bin)
                ]

                # Compute average z for the current bin
                if bin_points:
                    transformed_z = [z for (_, _, z) in bin_points]
                    avg_z = np.mean(transformed_z)
                else:
                    avg_z = 0

                # Append bar center positions and heights
                x_center.append((x_min_bin + x_max_bin) / 2)
                y_center.append((y_min_bin + y_max_bin) / 2)
                dz.append(avg_z)

                # draw 2d plot
                self.dot_2d_plt(bin_points, x_offset, y_offset, x_bins, y_bins, i, j, x_split)

        x_center = np.array(x_center)
        y_center = np.array(y_center)
        dz = np.array(dz)
        # dz = np.clip(dz, 0, zmax - zmin)

        # Normalize dz values for color mapping
        norm = plt.Normalize(dz.min(), dz.max())
        cmap = plt.get_cmap('autumn')  # Color map from yellow to red
        colors = cmap(norm(dz))

        # Create 3D bar plot
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='3d')
        ax.bar3d(x_center, y_center, 0, dx, dy, dz, color=colors, shade=True)

        # Set axis ticks
        ax.set_xticks(np.arange(xmin, xmax + x_split, x_split))
        ax.set_yticks(np.arange(ymin, ymax + y_split, y_split))
        ax.set_zticks(np.arange(zmin, zmax + z_split, z_split))

        def on_key(event):
            if event.key == 'q':
                plt.close(fig)

        fig.canvas.mpl_connect('key_press_event', on_key)

        if self.init_bool:
            self.init_workload = self.calculate_workload(dx, dy, dz)
            self.percent_workload = 100 - (self.init_workload / self.init_workload) * 100
            self.init_bool = False
            print(f"Volume: {self.init_workload: .3f}")

        else:
            current_workload = self.calculate_workload(dx, dy, dz)
            self.percent_workload = 100 - (current_workload / self.init_workload) * 100
            print(f"Volume: {current_workload: .3f}")

        if self.percent_workload < 0.0:
            self.percent_workload = 0.0
        # elif self.percent_workload > 100.0:
        #     self.percent_workload - 100.0

        print(f"Work progress: {self.percent_workload: .3f}")
        end = time.time()
        print(f"Program time: {end - start: .3f} sec")
        plt.show()
        return


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    return


if __name__ == '__main__':
    main()
