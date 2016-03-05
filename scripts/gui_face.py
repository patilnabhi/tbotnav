#!/usr/bin/python

from Tkinter import *
import os

root = Tk(className = 'train_faces_gui')
svalue = StringVar() # defines the widget state as string

w = Entry(root,textvariable=svalue) # adds a textarea widget
w.pack()

def act():
    name = svalue.get()
    os.system('rosrun tbotnav train_faces.py %s'%name)
    
foo = Button(root,text="Train", command=act)
foo.pack()
root.mainloop()