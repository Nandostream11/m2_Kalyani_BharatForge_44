import cv2
import torch
import torchvision
from torchvision.models.detection import fasterrcnn_resnet50_fpn
from torchvision.models.detection.faster_rcnn import FasterRCNN_ResNet50_FPN_Weights

def main():
    # Load pre-trained Faster R-CNN model
    model = fasterrcnn_resnet50_fpn(weights=FasterRCNN_ResNet50_FPN_Weights.DEFAULT)
    model.eval()

    # COCO labels dictionary
    COCO_LABELS = {
        0: "__background__",
        1: "person", 2: "bicycle", 3: "car", 4: "motorcycle", 5: "airplane",
        6: "bus", 7: "train", 8: "truck", 9: "boat", 10: "traffic light",
        # Add other COCO classes as needed
    }

    # Open webcam
    cap = cv2.VideoCapture(0)

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            break

        # Prepare image for model
        transform = torchvision.transforms.Compose([
            torchvision.transforms.ToTensor()
        ])
        image_tensor = transform(frame).unsqueeze(0)

        # Perform object detection
        with torch.no_grad():
            outputs = model(image_tensor)[0]

        # Draw detections
        for box, score, label in zip(outputs['boxes'], outputs['scores'], outputs['labels']):
            if score >= 0.5:  # Confidence threshold
                x1, y1, x2, y2 = box.int().tolist()
                label_name = COCO_LABELS[label.item()]
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{label_name}: {score.item():.2f}", 
                            (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

        # Display the resulting frame
        cv2.imshow('Object Detection', frame)

        # Break loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release resources
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
