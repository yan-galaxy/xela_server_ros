# 使用用户提供的参数设置保存文件夹名称
DATA_NUM="$1"
cp -rpd /home/galaxy/Desktop/Xela_ws/src/threed_viz/data/ /home/galaxy/Desktop/Xela_ws/src/threed_viz/storage_data/"$1"/

cp -rpd /home/galaxy/Desktop/Xela_ws/src/threed_viz/video/output.mp4 /home/galaxy/Desktop/Xela_ws/src/threed_viz/storage_data/"$1"/
