#Code to know the parameters of a function
import inspect
import pybullet as p


from RobotDataBaseClass import RobotDataBase

ex_datab = RobotDataBase()
print(dir(ex_datab))
print("\n"*4)
print(ex_datab.__dict__)
print("\n"*2)
print(ex_datab.__doc__)


#inspect.getargspec(print())
#help(print)
#help(p.changeDynamics)
#print(dir(print))


"""
print(dir(p.changeDynamics))
print("\n"*2)
print(p.changeDynamics.__doc__)
print("\n"*2)
print(p.changeDynamics.__getattribute__)
print("\n"*2)
print(p.changeDynamics.__module__)
print("\n"*2)
print(p.changeDynamics.__name__)
print("\n"*2)
print(p.changeDynamics.__qualname__)
print("\n"*2)
print(p.changeDynamics.__self__)
print("\n"*2)
print(p.changeDynamics._defaults__[0])
"""
