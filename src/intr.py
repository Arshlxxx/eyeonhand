import pyrealsense2 as rs

# 1. 配置 RealSense 管道
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # 设置流的格式、分辨率和帧率

# 2. 开始流
pipeline_profile = pipeline.start(config)

# 3. 获取 color 传感器的内参
color_stream = pipeline_profile.get_stream(rs.stream.color)  # 获取 color 流
intrinsics = color_stream.as_video_stream_profile().get_intrinsics()  # 获取内参

# 4. 打印内参
print("Width:", intrinsics.width)
print("Height:", intrinsics.height)
print("fx:", intrinsics.fx)  # 焦距 x 方向
print("fy:", intrinsics.fy)  # 焦距 y 方向
print("cx:", intrinsics.ppx)  # 主点 x 坐标
print("cy:", intrinsics.ppy)  # 主点 y 坐标
print("畸变模型:", intrinsics.model)  # 畸变模型
print("畸变系数:", intrinsics.coeffs)  # 畸变系数

# 5. 停止流
pipeline.st
