#uniquely identifies this blob in the system
int32 id

#array structure: {blue1, green1, red1, blue2, green2,...}
uint8[] colors

#the sequence number in which the blob was first added to the chain
int32 first_seen

#the most recent sequence number in which the blob was seen
int32 last_seen

#the upper left corner of the position, structure: {x,y}
int32[2] filtered_position

#the width and height of the position, structure: {width,height}
int32[2] filtered_size

#the centroid, structure: {x,y}
int32[2] centroid

#the list of points in the contour, structure: {x1,y1,x2,y2,...}
int32[] contour

#the current depth position
uint8 depth