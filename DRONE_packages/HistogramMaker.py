import numpy as np
import matplotlib.pyplot as plt
import math


# === HistogramMaker: Builds polar histograms from certainty grid ===
class HistogramMaker:
    def __init__(self, board_ref, sector_count=360, weight_constant=100, smoothing_k=5):
        self.board = board_ref
        self.sector_count = sector_count
        self.C = weight_constant
        self.K = smoothing_k
        self.board_size = self.board.shape[0]
        self.center = self.board_size // 2
    def update_polar_histogram(self):
        #Computes and smooths the 1D polar histogram
        histogram = np.zeros(self.sector_count, dtype=float)
        for i in range(self.board_size):
            for j in range(self.board_size):
                if self.board[i, j] > 0:
                    x = i - self.center
                    y = j - self.center
                    distance = math.sqrt(x**2 + y**2)
                    if distance > 0:
                        sector = int((math.degrees(math.atan2(y, x)) + 360) % 360 / (360 / self.sector_count))
                        histogram[sector] += self.C * self.board[i, j] / (distance**2)
                        # histogram[sector] += C * (board[i, j] ** 2) * (max(0, CENTER - distance))  
                        # histogram[sector] += C * (board[i, j] ** 2)/distance

        smoothed_histogram= np.zeros(self.sector_count, dtype=float)
        for i in range(self.sector_count):
            total = 0
            weight_sum = 0
            for offset in range(-self.K, self.K + 1):
                index = (i + offset) % self.sector_count
                weight = self.K + 1 - abs(offset)
                total += histogram[index] * weight
                weight_sum += weight
            smoothed_histogram[i] = total / weight_sum if weight_sum > 0 else 0

        return smoothed_histogram
    def binarize_histogram(self, histogram, threshold_factor=0.2):
        threshold = threshold_factor * max(histogram)
        binary = (np.array(histogram) > threshold).astype(int)
        return binary
    def plot_histogram(self, histogram, angle=None, magnitude=None):
        plt.figure(1)
        plt.clf()
        x = np.arange(len(histogram))
        plt.bar(x * 2, histogram)
        if angle is not None:
            plt.bar(int(angle), magnitude or 1, color='green')
        plt.xlabel("Angle")
        plt.ylabel("Certainty of Obstacle")
        plt.title("Smoothed Histogram")
        plt.pause(0.1)

        if angle is not None:
            #plt.figure(2)   put in stream_histogram
            plt.clf()
            ax = plt.gca()
            ax.set_xlim(-10, 10)
            ax.set_ylim(-10, 10)
            ax.set_xticks([])
            ax.set_yticks([])
            ax.set_frame_on(False)
            angle_rad = np.radians(angle)
            x_end = (magnitude or 1) * np.cos(angle_rad)
            y_end = (magnitude or 1) * np.sin(angle_rad)
            segments = [
                [0, 0, magnitude or 1, angle_rad],
                [x_end, y_end, (magnitude or 1) / 10, np.radians((angle + 205) % 360)],
                [x_end, y_end, (magnitude or 1) / 10, np.radians((angle + 155) % 360)],
            ]
            for x_start, y_start, length, ang in segments:
                x_end = x_start + length * np.cos(ang)
                y_end = y_start + length * np.sin(ang)
                plt.plot([x_start, x_end], [y_start, y_end], 'r-', linewidth=2)
            plt.pause(0.1)
