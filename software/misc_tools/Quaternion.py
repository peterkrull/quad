
from math import sqrt

class Quaternion:
    
    def __init__(self, nw = 1.0, nx = 0.0, ny = 0.0, nz = 0.0) -> None:
        self.w = nw
        self.x = nx
        self.y = ny
        self.z = nz

    def getProduct(self,q:'Quaternion') -> 'Quaternion' :
        return Quaternion(
            self.w*q.w - self.x*q.x - self.y*q.y - self.z*q.z, # new w
            self.w*q.x + self.x*q.w + self.y*q.z - self.z*q.y, # new x
            self.w*q.y - self.x*q.z + self.y*q.w + self.z*q.x, # new y
            self.w*q.z + self.x*q.y - self.y*q.x + self.z*q.w) # new z

    def getConjugate(self) -> 'Quaternion' :
        return Quaternion(self.w, -self.x, -self.y, -self.z)
    
    def getMagnitude(self) -> float :
        return sqrt(self.w*self.w + self.x*self.x + self.y*self.y + self.z*self.z) 

    def getConNorm (self) -> 'Quaternion' :
        return self.getConjugate().getNormalized()
    
    def normalize(self):
        m = self.getMagnitude()
        self.w /= m
        self.x /= m
        self.y /= m
        self.z /= m
    
    def getNormalized(self) -> 'Quaternion' :
        r = Quaternion(self.w, self.x, self.y, self.z)
        r.normalize()
        return r
    

class VectorFloat :

    def __init__(self, nx = 0.0, ny = 0.0, nz = 0.0 ) -> None :
        self.x = nx
        self.y = ny
        self.z = nz
    
    def subtract(self,num : 'VectorFloat') -> 'VectorFloat' :
        return VectorFloat (
            self.x - num.x,
            self.y - num.y,
            self.z - num.z
        )

    def add(self,num : 'VectorFloat') -> 'VectorFloat' :
        return VectorFloat (
            self.x + num.x,
            self.y + num.y,
            self.z + num.z
        )

    def getMagnitude(self) -> float :
        return sqrt(self.x*self.x + self.y*self.y + self.z*self.z)

    def normalize(self) :
        m = self.getMagnitude()
        self.x /= m
        self.y /= m
        self.z /= m
    
    def getNormalized(self) -> 'VectorFloat' :
        r = VectorFloat(self.x, self.y, self.z)
        r.normalize()
        return r
    
    def rotate(self,q : Quaternion) :
        p = Quaternion(0, self.x, self.y, self.z)

        # // quaternion multiplication: q * p, stored back in p
        p = q.getProduct(p)

        # // quaternion multiplication: p * conj(q), stored back in p
        p = p.getProduct(q.getConjugate())

        # // p quaternion is now [0, x', y', z']
        self.x = p.x
        self.y = p.y
        self.z = p.z

    def getRotated(self,q : Quaternion) -> 'VectorFloat' :
        r = VectorFloat(self.x, self.y, self.z);
        r.rotate(q)
        return r

    def getFraction(self,f : float) -> 'VectorFloat' :
        r = VectorFloat(self.x, self.y, self.z)
        return VectorFloat( r.x/f , r.y/f , r.z/f )

    def getProduct(self, f : float) -> 'VectorFloat' :
        r = VectorFloat(self.x, self.y, self.z)
        return VectorFloat( r.x*f , r.y*f , r.z*f )
    
v1 = VectorFloat(1,0,0)
v2 = VectorFloat(0,1,0)
v3 = VectorFloat(0,0,1)

vx = VectorFloat(1,1,1)

q = Quaternion(1,3,0,0).getNormalized()

qv1 = v1.getRotated(q)
qv2 = v2.getRotated(q)
qv3 = v3.getRotated(q)
qvx = vx.getRotated(q)

print(f"qv1 : [{qv1.x} , {qv1.y} , {qv1.z}]")
print(f"qv2 : [{qv2.x} , {qv2.y} , {qv2.z}]")
print(f"qv3 : [{qv3.x} , {qv3.y} , {qv3.z}]")
print(f"qvx : [{qvx.x} , {qvx.y} , {qvx.z}]")