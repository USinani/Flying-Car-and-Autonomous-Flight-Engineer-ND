import numpy as np
import matplotlib.pyplot as plt

def first_method():
    print('hello from first method')
    x = 12
    y = 15
    total = x + y
    return print(total)

def second_method():
    print('hello from second method')

def third_method():
    print('hello from third method')

# Python built-in functions

def builtin_functions():
    # print function
    objects = 3
    print('hello',objects)
    
    # list function
    
    # open function

'''
class Rectangle:
   def __init__(self, length, breadth, unit_cost=0):
       self.length = length
       self.breadth = breadth
       self.unit_cost = unit_cost
   def get_area(self):
       return self.length * self.breadth
   def calculate_cost(self):
       area = self.get_area()
       return area * self.unit_cost
# breadth = 120 units, length = 160 units, 1 sq unit cost = Rs 2000
r = Rectangle(160, 120, 2000)
print("Area of Rectangle: %s sq units" % (r.get_area()))

class Square:
    def __init__(self, length, height, unit_cost = 3):
        self.length = length
        self.height = height
        self.unit_cost = unit_cost

    def calculate_area(self):
        return self.length * self.height
        
    def calculate_cost(self):
        area = self.calculate_area()
        return area * self.unit_cost

s = Square(234, 34, 354)
print('Area of Square',  (s.calculate_area()) )
'''