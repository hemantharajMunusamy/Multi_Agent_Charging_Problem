import math

"""
Point Class contain data - (x, y)
and operation like +, - , *, dot, display
"""
class Point:
    """Random variable generators.

    bytes
    -----
           uniform bytes (values between 0 and 255)

    integers
    --------
           uniform within range

    sequences
    ---------
           pick random element
           pick random sample
           pick weighted random sample
           generate random permutation

    distributions on the real line:
    ------------------------------
           uniform
           triangular
           normal (Gaussian)
           lognormal
           negative exponential
           gamma
           beta
           pareto
           Weibull

    distributions on the circle (angles 0 to 2pi)
    ---------------------------------------------
           circular uniform
           von Mises

General notes on the underlying Mersenne Twister core generator:

* The period is 2**19937-1.
* It is one of the most extensively tested generators in existence.
* The random() method is implemented in C, executes in a single Python step,
  and is, therefore, threadsafe.

"""
    def __init__(self, x, y):
        self.x = x
        self.y = y
        
    def __add__(self, other):
        """
        Add two point
        """
        temp = Point(self.x, self.y)
        
        temp.x += other.x
        temp.y += other.y
        
        return temp
    
    def __sub__(self, other):
        """
        Subtract two point
        """
        temp = Point(self.x, self.y)
        
        temp.x -= other.x
        temp.y -= other.y
        
        return temp

    def __mul__(self, alpha):
        """
        Multiply with alpha
        """
        temp = Point(self.x, self.y)
        
        temp.x = alpha * temp.x
        temp.y = alpha * temp.y
        
        return temp
    
    def __rmul__(self, alpha):
        """
        Multiply scalar with point
        """
        temp = Point(self.x, self.y)
        
        temp.x = alpha * temp.x
        temp.y = alpha * temp.y
        
        return temp
    
    def dot(self, other):
        """
        Dot product 
        """
        temp = Point(self.x, self.y)
        
        norm = temp.x * other.x + temp.y * other.y
        
        return norm
    
    """
    Consider point has vector form, rotate the vector according rotation matrix
    """
    def rotate(self, angle):
        rot_pos = Point(0., 0.)
        rot_pos.x = math.cos(angle) * self.x - math.sin(angle) * self.y
        rot_pos.y = math.sin(angle) * self.x + math.cos(angle) * self.y
        
        return rot_pos

    """
    Display the point
    """  
    def __str__(self):   
        return "x = " + str(round(self.x,2)) + " y = " + str(round(self.y,2))
    
    

"""
Line class contains straight line data - xi(x,y) and xd(x,y)
xi - initial location and xd - difference between initial and final
And functionality required for finiding collision
"""
class Line:
    def __init__(self, xi, yi, xf, yf):
        #initial point
        self.xi = Point(xi, yi)
        #final point
        self.xf = Point(xf, yf)
        #directional point
        self.xd = Point(xf - xi, yf - yi)

    """
    Get path length
    """
    def get_distance(self):
        return math.sqrt(self.xd.dot(self.xd))
    """
    check line with pt and return True if it is collinear
    """    
    def is_collinear(self, pt):
        #If three points (x1, y1), (x2, y2) and (x3, y3) are collinear then [x1(y2 - y3) + x2( y3 - y1)+ x3(y1 - y2)] = 0
        val = (self.xi.x * (self.xf.y - pt.y)) + (self.xf.x * (pt.y - self.xi.y)) + (pt.x * (self.xi.y - self.xf.y))
    
        if abs(val) < 0.00001:
            return True
        else :
            return False
        
    """
    Given two line segment, check collision 
    """
    def get_collision_dist(self,l2):
        det = (-self.xd.x * l2.xd.y) + (self.xd.y * l2.xd.x) 
        #two lines are parallel
        if abs(det) < 0.000001:   
            #choose any point to another line to find minimum distance
            dist = self.min_dist(l2, 0.0)
            return (2, dist)
        #two lines are crossing
        else:
            #check two lines crossing within line segment
            #alpha value needs to be within [0, 1] for collision within segment
            #Either alpha more than bounds, minimum collision distance is in end-point 
            alpha_1 = (-l2.xd.y *(l2.xi.x - self.xi.x)) + (l2.xd.x *(l2.xi.y - self.xi.y))
            alpha_1 = alpha_1 / det
            
            alpha_2 = (-self.xd.y *(l2.xi.x - self.xi.x)) + (self.xd.x * (l2.xi.y - self.xi.y))
            alpha_2 = alpha_2 / det

            #truncate both alpha [minimum distance in the endpoint ]
            if (alpha_1 > 1.0 or alpha_1 < -0.000001) and (alpha_2 > 1.0 or alpha_2 < -0.000001):
                if alpha_1 > 1.0:
                    alpha_1 = 1.0
                elif alpha_1 < -0.000001:
                    alpha_1 = 0.0
                    
                if alpha_2 > 1.0 :
                    alpha_2 = 1.0
                elif alpha_2 < -0.000001:
                    alpha_2 = 0.0
           
                dist1 = l2.min_dist(self, alpha_1)
                dist2 = self.min_dist(l2, alpha_2)
                
                if dist1 < dist2:
                    return (0, dist1)
                else:
                    return (0, dist2)   
            elif (alpha_1 > 1.0 or alpha_1 < -0.00001):
                if alpha_1 > 1.0:
                    alpha_1 = 1.0
                elif alpha_1 < 0.0:
                    alpha_1 = 0.0
                    
                return (0,l2.min_dist(self, alpha_1))   
            elif (alpha_2 > 1.0 or alpha_2 < -0.000001):    
                if alpha_2 > 1.0:
                    alpha_2 = 1.0
                elif alpha_2 < 0.0:
                    alpha_2 = 0.0
                
                return (0, self.min_dist(l2, alpha_2))
            #there is collision because both alphas between [0,1]
            else:
                return (1, 0.0)

    """
    Return Minimum distance between line and point (l2, alpha)
    """
    def min_dist(self, l2, alpha):
        norm = math.sqrt(self.xd.dot(self.xd))
        if abs(norm) < 0.001:
            #single point
            pt1 = Point(self.xi.x, self.xi.y)
            pt2 = l2.xi + (alpha * l2.xd)
            return math.sqrt((pt1.x - pt2.x)**2 + (pt2.y - pt2.y)**2) 
        else:
            #get point from l2
            p3 = l2.xi + (alpha * l2.xd)

            p1_p3 = p3 - self.xi
            p2_p3 = p3 - (self.xi + self.xd)
        
            #find unit vector along the first line
            norm_xd = Point(self.xd.x, self.xd.y)

            norm_xd.x = norm_xd.x / norm
            norm_xd.y = norm_xd.y / norm

            #find adjacent length 
            adj1 = norm_xd.dot(p1_p3)
            adj2 = norm_xd.dot(p2_p3)
            
            if adj1 * adj2 < 0.0:
                #point in between line segment (/-)
                #use triangle equality
                dist = p1_p3.dot(p1_p3) - (adj1 * adj1)
                dist = abs(dist)
                return math.sqrt(dist)
            else:
                #point is more close with line either corner point (/' or /,)
                dist1 = math.sqrt(abs(p1_p3.dot(p1_p3)))
                dist2 = math.sqrt(abs(p2_p3.dot(p2_p3)))
            
                if dist1 < dist2:
                    return dist1
                else:
                    return dist2
        
     
    def __str__(self):
        return "xi => "+ str(self.xi) + " xd => " + str(self.xd) + " xf => " + str(self.xf)