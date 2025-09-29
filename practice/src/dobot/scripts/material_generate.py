import numpy as np
from PIL import Image

# 定义棋盘格参数
square_size = 50  # 每个方块的像素大小
board_size = 8    # 棋盘格的方块数量（8x8）
color1 = (255, 255, 255)  # 白色
color2 = (0, 0, 0)        # 黑色

# 创建空白图像
img_size = square_size * board_size
img = np.zeros((img_size, img_size, 3), dtype=np.uint8)

# 填充棋盘格
for i in range(board_size):
    for j in range(board_size):
        if (i + j) % 2 == 0:
            img[i*square_size:(i+1)*square_size, j*square_size:(j+1)*square_size] = color1
        else:
            img[i*square_size:(i+1)*square_size, j*square_size:(j+1)*square_size] = color2

# 保存图像
Image.fromarray(img).save('/home/linkedata/projects/ros2/ros2_test/ros2_learn/practice/src/dobot/textures/chess_board.png')
