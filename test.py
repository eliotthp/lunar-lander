from dataclasses import dataclass
import numpy as np


class Dog:
    def __init__(self, name, age):
        self.name = name
        self.age = age

    def bark(self):
        print("Woof!")

    def __str__(self):
        return f"{self.name} is {self.age} years old"


Dog1 = Dog("Mooshu", 5)
Dog1.bark()
print(Dog1)


@dataclass
class VehicleInfo:
    state: np.ndarray

    @property
    def x(self):
        return self.state[2]

    @property
    def inverse_x(self):
        return -self.x


LM = VehicleInfo(
    np.array(
        [
            10,
            20,
            30,
            -10,
            500,
        ]
    )
)

print(LM.inverse_x)
