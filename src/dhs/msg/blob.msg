Header header

#uniquely identifies this blob in the system
int32 id

#array structure: {blue1, green1, red1, blue2, green2,...}
uint8[] colors

#the sequence number in which the blob was first added to the chain
int32 first_seen

#the most recent sequence number in which the blob was seen
int32 last_seen

#the upper left corner of the raw position, structure: {x,y}
int32[2] raw_position

#the width and height of the raw position, structure: {width,height}
int32[2] raw_size

#the upper left corner of the filtered position, structure: {x,y}
int32[2] filtered_position

#the width and height of the filtered position, structure: {width,height}
int32[2] filtered_size

#the moments, structure: m00, m10, m01, m20, m11, m02, m30, m21, m12, m03 
int32[10] moments

#the list of points in the contour, structure: {x1,y1,x2,y2,...}
int32[] contour

#the current depth position
uint8 depth