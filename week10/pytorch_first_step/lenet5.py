import torch
import torch.nn as nn

#Lenet5 직접 구현
class Lenet5(nn.Module):
    def __init__(self, batch, n_classes, in_channel, in_width, in_height, is_train = False):
        super().__init__()
        self.batch = batch
        self.n_classes = n_classes
        self.in_channel = in_channel
        self.in_width = in_width
        self.in_height = in_height
        self.is_train = is_train

        # convolution output : [(W-K+2P)/S]+1(정사각형 이미지 공식)
        #(32-5+2*0)/1 + 1
        self.conv0 = nn.Conv2d(self.in_channel, 6, kernel_size=5, stride=1, padding=0)
        self.pool0 = nn.AvgPool2d(2,stride=2)
        self.conv1 = nn.Conv2d(6,16, kernel_size=5, stride=1, padding=0)
        self.pool1 = nn.AvgPool2d(2,stride=2)
        self.conv2 = nn.Conv2d(16,120, kernel_size=5, stride=1, padding=0)

        #fully-connected layer
        self.fc3 = nn.Linear(120,84)
        self.fc4 = nn.Linear(84, self.n_classes)

    def forward(self,x):
        #x shape : [Batch, Channel,H,W]
        x=self.conv0(x)
        x=torch.tanh(x)
        x=self.pool0(x)
        #x = nn.functional.avg_pool2d(x,kernel_size=2,stride=2)
        x=self.conv1(x)
        x=torch.tanh(x)
        x=self.pool1(x)
        #x = nn.functional.avg_pool2d(x,kernel_size=2, stride=2)
        x=self.conv2(x)
        x=torch.tanh(x)
        #change format from 4dim -> 2dim([B,C,H,W]->[B,C*H*W])
        x =torch.flatten(x, start_dim=1)
        x= self.fc3(x)
        x=torch.tanh(x)
        x=self.fc4(x)
        x= x.view(self.batch, -1)
        x = nn.functional.softmax(x, dim=1)
        if self.is_train is False:
            x=torch.argmax(x, dim=1)
        return x