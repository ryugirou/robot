#!/usr/bin/env python
# coding: UTF-8
from enum import IntEnum

#スーパークラス
class SuperClass(object): #objectクラスを継承するのを忘れない
	def __init__(self):
		self.__name ="Taro" #nameを初期化
		
#サブクラス
class SubClass(SuperClass): #SuperClassを継承する
	def __init__(self):
		super(SubClass, self).__init__() #super()を使ってスーパークラスの__init__()を呼び出す
 
if __name__ == '__main__':
 sup = SuperClass()
 sub = SubClass()
 print(sub._SuperClass__name) #Taro
#  print(sup._SuperClass__name) #Taro