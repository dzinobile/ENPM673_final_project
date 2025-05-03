import cv2
import torch

# Load YOLOv5 model (using yolov5s pretrained)
model = torch.hub.load('yolov5', 'yolov5s', source='local')
model.eval()

# Open webcam (use 0 for default camera)
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Run inference
    results = model(frame)
    labels = results.names
    detections = results.pred[0]
    stop_sign_detected = False

    for *box, conf, cls in detections:
        label = labels[int(cls)]
        if label.lower() == "stop sign":
            stop_sign_detected = True
            x1, y1, x2, y2 = map(int, box)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.putText(frame, f"{label} ({conf:.2f})", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

    # Show the result and boolean output
    print("Stop sign detected:", stop_sign_detected)
    cv2.imshow('Stop Sign Detector', frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
exit()
