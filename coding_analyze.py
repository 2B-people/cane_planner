#!/usr/bin/python
# -*- coding:utf-8 -*-
import os
import time
import sys

# 统计代码量
# 使用len(t_coding)function选择统计对象
baseroot = os.getcwd()
whiteDict = {'py':['py'],'cpp&c':['cpp','h','hpp','c'],'web':['html','css','ts','xml'],'matlab':['m']}
FileList = []

def afileline(f_path):
    count = 0
    for file_line in open(f_path,'rb').readlines():
        if file_line != '' and file_line != '\n': #过滤掉空行
            count += 1
    return count

def Readfile(t_coding):
    for root,dirs,files in os.walk(baseroot):
        for t_file in files:
            ext = t_file.split('.')[-1]
            if ext in whiteDict[t_coding]:
                FileList.append(root+os.sep+t_file)

def len(t_coding):
    # s_time = time.clock()
    Readfile(t_coding)
    sum = 0
    for file in FileList:
        sum += afileline(file)
    print('program line sum is:',sum)    
    # print('Done! Cost time is ',(time.clock()-s_time),'second')
    FileList.clear()


# eg，Timer类，记时测量
# 为了使用它，你需要用Python的with关键字和Timer上下文管理器包装想要计时的代码块。
# 它将会在你的代码块开始执行的时候启动计时器，在你的代码块结束的时候停止计时器。 
# from timer import Timer
# from redis import Redis
# rdb = Redis()
# with Timer() as t:
#     rdb.lpush("foo", "bar")
# print "=> elasped lpush: %s s" % t.secs
# with Timer as t:
#     rdb.lpop("foo")
# print "=> elasped lpop: %s s" % t.secs

# class Timer(object):
#     def __init__(self, verbose=True):
#         self.verbose = verbose

#     def __enter__(self):
#         self.start = time.clock()
#         return self

#     def __exit__(self, *args):
#         self.end = time.clock()
#         self.secs = self.end - self.start
#         self.msecs = self.secs * 1000  # millisecs
#         if self.verbose:
#             print ('elapsed time: ',self.msecs,'ms')

if __name__ == "__main__":
    len('cpp&c')
    len('py')
