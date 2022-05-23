import torch
import numpy as np
def make_tensor():
    a = torch.tensor([[1,2],[3,4]],dtype=torch.int16)

    b = torch.tensor([3.14],dtype=torch.float32)

    c = torch.tensor([1.23],dtype=torch.float64)        # double 형식

    print(a,b,c)
    tensor_list =[a,b,c]

    for t in tensor_list:
        print("shape of tensor {}".format(t.shape))                 # shape : tensor의 크기 출력, numpy와 동일
        print("datatype of tensor {}".format(t.dtype))              # dtype : dtype 출력
        print("device tensor is stored on {}".format(t.device))     # device : cpu, gpu 등 가동장치 표시           

def sumsub_tensor():
    a= torch.tensor([3,2])
    b = torch.tensor([5,3])

    print("input {} , {}".format(a,b))

    sum = a+b
    print("sum : {}".format(sum))

    sub = a-b
    print("sub : {}".format(sub))

    sum_element_a = a.sum()     # 원소 모두 sum
    print(sum_element_a)

def muldiv_tensor():
    a= torch.arange(0,9).view(3,3)      # view는 numpy의 reshape와 동일
    #b = torch.zeros(9).view(3,3)
    b = torch.arange(0,9).view(-1,3)
    print("input tensor {}, {}".format(a,b))

    c=torch.matmul(a,b)                 #matmul : 행렬곱
    print(c)

    d= torch.mul(a,b)               # 각 인덱스에 해당하는 곱만 진행
    print(d)

def reshape_tensor():
    a = torch.tensor([2,4,5,6,7,8])
    print("input tensor : \n {}".format(a))

    b= a.view(2,3)
    print("view \n {}".format(b))

    bt= b.t()        # t : transpose
    print("transpose \n {}".format(bt))

def access_tensor():
    a = torch.arange(1,13).view(4,3)
    print("input : \n {}".format(a))

    print(a[:,0])

    print(a[0,:])

    print(a[1,2])

def transform_numpy():
    a = torch.arange(1,13).view(4,3)
    print("input : \n {}".format(a))

    a_np = a.numpy()
    print("numpy : \n {}".format(a_np))

    b= np.array([1,2,3])
    bt = torch.from_numpy(b)
    print(bt)

def concat_tensor():
    a = torch.arange(1,10).view(3,3)
    b = torch.arange(10,19).view(3,3)
    c = torch.arange(19,28).view(3,3)

    abc = torch.cat([a,b,c],dim=0)

    print("inpput tensor : \n {} \n{} \n{}".format(a,b,c))
    print("concat : \n {}".format(abc))
    print(abc.shape)

def stack_tensor():
    a = torch.arange(1,10).view(3,3)
    b = torch.arange(10,19).view(3,3)
    c = torch.arange(19,28).view(3,3)

    abc = torch.stack([a,b,c],dim=2)    # dim 0 : 차원 추가, 그대로 나열
                                        # dim 1 : 각 행렬마다 행 단위 입력, 이후 출력
                                        # dim 2 : 각 행렬마다 열 단위 입력, 이후 출력
    
    print("inpput tensor : \n {} \n{} \n{}".format(a,b,c))
    print("concat : \n {}".format(abc))
    print(abc.shape)

def transpose_tensor():
    a = torch.arange(1,10).view(3,3)
    print("input tensor : \n {}".format(a))

    at = torch.transpose(a,0,1)
    print("transpose : \n {}".format(at))

    b = torch.arange(1,25).view(4,3,2)
    print("input b tensor : \n {}".format(b))

    bt = torch.transpose(b,0,2)
    print("transpose : \n {}".format(bt))
    print(bt.shape)

    bp = b.permute(2,0,1)
    print("permute : \n {}".format(bp))
    print(bp.shape)
    

def Quiz1():
    a= torch.arange(1,7).view(2,3)
    b= torch.arange(1,7).view(2,3)
    
    print("a : \n {}".format(a))
    print("b : \n {}".format(b))
    print("sum : \n {} ".format(a+b))
    print("sub : \n {} ".format(a-b))
    print("sum_all_elements_a : {} ".format(a.sum()))
    print("sum_all_elements_b : {} ".format(b.sum()))
    

def Quiz2():
    a=torch.arange(1,46).view(1,5,3,3)
    at = torch.transpose(a,1,3)
    print("transpose : \n {}".format(at))
    print(at[0,2,2,:])

def Quiz3():
    a=torch.arange(1,7).view(2,3)
    b=torch.arange(1,7).view(2,3)  

    con_ab= torch.concat([a,b],dim=1)
    print("concat : \n {}".format(con_ab))

    stack_ab= torch.stack([a,b],dim=0)
    print("stack : \n {}".format(stack_ab))


if __name__ == "__main__":
    #make_tensor()
    #sumsub_tensor()
    #muldiv_tensor()
    #reshape_tensor()
    #access_tensor()
    #transform_numpy()
    #concat_tensor()
    #stack_tensor()
    #transpose_tensor()
    #Quiz1()
    #Quiz2()
    Quiz3()