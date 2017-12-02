# -*- coding: UTF-8 -*-
#排列组合和阶乘函数
def factorial(number):
    result=1
    for i in range(1,number):
        result=i*result
    print ('result is %d'%result)
def permute(list):

    #接受任何序列
    if not list:
        #返回空序列
        return [list]
    else:
        res=[]
        for i in range(len(list)):
            #删除当前节点
            rest=list[:i]+list[i+1:]
            #排列其他的节点
            for x in permute(rest):
                #把当前节点添加到前面
                res.append(list[i:i+1]+x)
        return res


factorial(13)
print (permute([1,2,3,4,5,6,7,8,9,0]))