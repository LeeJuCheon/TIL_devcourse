import torch
import torch.nn as nn
import sys
import os
import argparse
from torch.utils.data.dataloader import DataLoader
from torchvision.datasets import MNIST
import torchvision.transforms as transforms
from models import *
import torch.optim as optim
from loss import *
from utils.tools import *

def parse_args():           #cmd에서 출력할 때 해당 명령어를 입력하면 설명을 출력
    parser = argparse.ArgumentParser(description="MNIST")
    parser.add_argument('--mode',dest="mode", help="train / eval / test",
                        default=None, type=str)
    parser.add_argument('--download',dest='download', help ="download MNIST dataset",
                        default=False, type=bool)
    parser.add_argument('--output_dir',dest='output_dir', help ="output directory",
                        default='./output_dir', type=str )
    parser.add_argument('--checkpoint', dest='checkpoint', help ="checkpoint trained model",
                        default=None, type=str)             

    if len(sys.argv) ==1 :
        parser.print_help()
        sys.exit()
    args = parser.parse_args()
    return args


def get_data():
    #image resize
    my_transform = transforms.Compose([
        transforms.Resize([32,32]),
        transforms.ToTensor(),
        transforms.Normalize((0.5,),(1.0,))
    ])           
    download_root = "./mnist_dataset"
    train_dataset = MNIST(download_root, 
                        transform=my_transform,
                        train=True,
                        download=args.download)
    eval_dataset = MNIST(root=download_root,
                        transform=my_transform,
                        train=False,
                        download=args.download)
    test_dataset = MNIST(root=download_root,
                        transform=my_transform,
                        train=False,
                        download=args.download)

    return train_dataset, eval_dataset, test_dataset

def main():
    print(torch.__version__)
    if not os.path.isdir(args.output_dir):
        os.mkdir(args.output_dir)

    if torch.cuda.is_available():
        print("gpu")
        device = torch.device("cuda:0")
    else:
        print("cpu")
        device = torch.device("cpu")

    #Get MNIST Dataset
    train_dataset, eval_dataset, test_dataset = get_data()
    
    #Make Dataloader
    train_loader= DataLoader(train_dataset,    
                             batch_size=8,
                             num_workers=0,     # 데이터 로드 멀티프로세싱,CPU 코어 개수
                             pin_memory=True,   # Tensor를 CUDA 고정 메모리에 올림
                             drop_last=True,    # batch단위로 쪼갤때 딱 떨어지지 않는 나머지 데이터는 버림
                             shuffle=True)      # shuffle : 순차적이 아닌 데이터를 섞어 랜덤성 부여

    eval_loader= DataLoader(train_dataset,    
                             batch_size=1,
                             num_workers=0,     
                             pin_memory=True,   
                             drop_last=False,    
                             shuffle=False)      

    test_loader= DataLoader(train_dataset,    
                             batch_size=1,
                             num_workers=0,     
                             pin_memory=True,   
                             drop_last=False,    
                             shuffle=False)      

    #LeNet5(1998 모델)
    _model = get_model("lenet5")


    if args.mode=="train":
        model = _model(batch=8, n_classes=10, in_channel=1, in_width=32, in_height=32, is_train=True)
        model.to(device)
        model.train()   

        #optimizer & scheduler
        
        optimizer = optim.SGD(model.parameters(), lr=0.01, momentum=0.9)       # 확률적 경사하강법
        scheduler = optim.lr_scheduler.StepLR(optimizer, step_size=5, gamma=0.1)     # 단계적으로 lr이 떨어짐
        
        criterion = get_criterion(crit='mnist', device=device)

        epoch = 15
        iter = 0
        for e in range(epoch):
            total_loss = 0
            for i,batch in enumerate(train_loader):
                img = batch[0]
                gt = batch[1]
                
                img = img.to(device)
                gt = gt.to(device)

                out = model(img)
                
                loss_val = criterion(out,gt)

                #back propagation
                loss_val.backward()
                optimizer.step()
                optimizer.zero_grad()

                total_loss+=loss_val.item()
                
                if iter%100 ==0:
                    print("{} epoch {} iter loss : {}".format(e,iter,loss_val.item()))
                iter+=1


            mean_loss=total_loss/i
            scheduler.step()
            print("->{} epoch mean loss : {}".format(e, mean_loss))
            torch.save(model.state_dict(),args.output_dir + "/model_epoch"+str(e)+".pt")
        print("Train end")

    elif args.mode=="eval":
        model = _model(batch=1, n_classees=10, in_channel=1,in_width=32,in_height=32)
        #load trained model
        checkpoint = torch.load(args.checkpoint)
        model.load_state_dict(checkpoint)
        model.to(device)
        model.eval()

        acc=0
        num_eval = 0

        for i, batch in enumerate(eval_loader):
            img = batch[0]
            gt = batch[1]

            img = img.to(device)

            #inference
            out = model(img)
            out = out.cpu()

            if out ==gt:
                acc+=1
            num_eval+=1

        print("Evaluation score : {}/ {}".format(acc,num_eval))
    elif args.mode=="test":
        model =_model(batch=1, n_classes=10, in_channel=1, in_width=1,in_height=1)
        checkpoint=torch.load(args.checkpoint)
        model.load_state_dict(checkpoint)
        model.to(device)
        model.eval()

        for i,batch in enumerate(test_loader):
            img = batch[0]

            img = img.to(device)

            # inference
            out = model(img)
            out = out.cpu()

            #show result
            show_img(img.cpu().numpy(),str(out.item()))

if __name__ == "__main__" :
    args= parse_args()
    main()