from ultralytics import YOLO
import cv2

def object_detection(model):
    """ Webカメラ物体検出 """
    cap = cv2.VideoCapture(1)
    
    cap.set(cv2.CAP_PROP_FPS, 90)
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))              
    # カメラの横幅を取得
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))             
    # カメラの縦幅を取得
    
    fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')        
    # 動画保存時のfourcc設定（mp4用）
    video = cv2.VideoWriter('yolov8-data/mypredict2/test.mp4', fourcc, fps, (w, h))  
    # 動画の仕様（ファイル名、fourcc, FPS, サイズ）

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        # フレームで物体検出
        results = model(frame)
        # アノテートされたフレームを表示
        annotated_frame = results[0].plot()
        cv2.imshow("YOLOv8!!!", annotated_frame)
        
        video.write(annotated_frame)  # 修正：ここでVideoWriterに書き込む
        
        # 'q'が押されたらループを終了
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    video.release()  # 修正：VideoWriterを解放
    cv2.destroyAllWindows()

if __name__ == '__main__':
    # YOLOモデルを読み込む
    model = YOLO('runs/detect/train62/weights/best.pt')
    # Webカメラのフレームで物体検出を実行
    object_detection(model)
