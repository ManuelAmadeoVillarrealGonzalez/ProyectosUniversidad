import cv2


def init_camera():
    video_capture = cv2.VideoCapture(0)
    ret = video_capture.set(3, 640)
    ret = video_capture.set(4, 480)
    return video_capture
    
def acquire_image(video_capture):
    # Grab a single frame of video
    ret, frame = video_capture.read()
    scaled_rgb_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
    scaled_rgb_frame = scaled_rgb_frame[:, :, ::-1]
    return frame, scaled_rgb_frame

def show_frame(frame):
    # Display the resulting image frame in the PAC
    cv2.imshow('Video',frame)

# Application dependent - Visualize face information
def draw_face_info_on_frame(frame, face_locations, face_names):
    for (top, right, bottom, left), name in zip(face_locations, face_names):
        # Scale back up face locations since the frame we detected in was scaled to 1/4 size
        top *= 4
        right *= 4
        bottom *= 4
        left *= 4
        # Draw a box around the face
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
        # Draw a label with a name below the face
        cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
        font = cv2.FONT_HERSHEY_DUPLEX
        cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)
