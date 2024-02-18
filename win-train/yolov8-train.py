from ultralytics import YOLO

def main():
    # model = YOLO('yolov8n.pt') # モデルを選択 #学習 
    # model.train(data='RedBalls.v2i.yolov8/data.yaml', epochs=100, batch=8, verbose=True)
    
    model = YOLO('runs/detect/train60/weights/best.pt')
    model.train(data='red-ball.yaml', epochs=40, batch=8, verbose=True)
    
if __name__ == '__main__':
    main()