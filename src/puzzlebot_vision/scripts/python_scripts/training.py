#only applicable for training yoloN and rfdetr, due to complexity for training yoloDamo the
#training method is only referenced for the main repository and not fully implemented below
#In the file requirements.txt there are the necessary dependencies for training both models, install them in a virtual environment to avoid
#conflicts with the ROS2 workspace dependencies
import glob
from argparse import ArgumentParser

def checkGPU():
    try: 
        import torch 
        print(f"{torch.cuda.is_available()}")
    except ImportError:
        print(f"Error: {ImportError}")

def trainYOLO():
    try:
        from ultralytics import YOLO

        model = YOLO('yolo26n.pt')
        results = model.train(
            data='dataset/data.yaml', 
            epochs=100, 
            imgsz=640, 
            batch=16, 
            device=0,
            optimizer="MuSGD",
            plots=True
        )
    except Exception:
        print(f"Error: {Exception}")

def trainRFDETR():
    try:
        from rfdetr import RFDERNano
        model.train(
            dataset_dir = 'dataset',
            epochs = 100,
            batch_size = 16,
            device = 'cuda',
            grad_acum_steps = 4,
            imgz=512,
            lr = 1e-4,
            early_stopping = False,
            output_dir = 'runs/train'
            #resume = 'models/rfdetr1.pt'
        )
    except Exception:
        print(f"Error: {Exception}")

def trainYOLODamo():
    """Please refer to the main repository for training yoloDamo"""
    #If encountered during the training process, due to uncompatibility
    #Ensure the numpy version is 1.26.4 or < 2.0 and reinstall pycocotools
    pass
    

def main():
    parser = ArgumentParser(description="Model")
    parser.add_argument("--model", type=str, default="yoloN", help="yoloN, yoloDamo, rfdetr")
    args = parser.parse_args()
    if args.model == "yoloN":
        trainYOLO()
    elif args.model == "rfdetr":
        trainRFDETR()
    else:
        print("Invalid model, options are yoloN, yoloDamo, rfdetr")


if __name__ == "__main__":
    main()