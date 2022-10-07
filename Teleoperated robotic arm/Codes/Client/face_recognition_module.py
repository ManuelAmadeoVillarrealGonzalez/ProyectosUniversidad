import face_recognition
import os
import numpy as np


KNOWN_FACES_DIR = 'known_faces'
TOLERANCE = 0.6
MODEL = 'cnn'


def load_known_faces_and_encode(dir_name):
    print('Loading known faces ...')
    known_face_encodings = []
    known_face_names = []
    for name in os.listdir(dir_name):
        for filename in os.listdir(f'{dir_name}/{name}'):
            image = face_recognition.load_image_file(f'{dir_name}/{name}/{filename}')
            encoding = face_recognition.face_encodings(image)[0]
            known_face_encodings.append(encoding)
            known_face_names.append(name)
    return known_face_encodings, known_face_names


def extract_faces_and_encode(rgb_image):
    # specify the model cnn or hog, the later is the deafault.
    face_locations = face_recognition.face_locations(rgb_image, model = MODEL)
    face_encodings = face_recognition.face_encodings(rgb_image,face_locations)
    return face_locations, face_encodings


def find_face_matches(face_encodings, known_face_encodings, known_face_names):
    face_names = []
    for face_encoding in face_encodings:
        name = "Unknown"
        # TOLERANCE = 0.6
        matches = face_recognition.compare_faces(known_face_encodings, face_encoding, TOLERANCE)
        # # If a match was found in known_face_encodings, just use the first one.
        # if True in matches:
        #     first_match_index = matches.index(True)
        #     name = known_face_names[first_match_index]
        # Or instead, use the known face with the smallest distance to the new face
        face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
        best_match_index = np.argmin(face_distances)
        if matches[best_match_index]:
            name = known_face_names[best_match_index]
        face_names.append(name)
    return face_names
