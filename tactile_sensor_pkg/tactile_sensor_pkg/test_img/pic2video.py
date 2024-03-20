import cv2
import os
folder_path ='/home/lifan/Documents/GitHub/Data_transfer/scripts/figs4/'
output_video_file = '/home/lifan/Documents/GitHub/Tactile-Stereo/test_img/pick_and_place_treatment/video_arrow/video/diagram14_update.mp4'

fps = 15

image = cv2.imread(folder_path+'fig_0.png')
# print(image)
# image = image[:,20:-30]

video_width = image.shape[1]
video_height = image.shape[0]

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
video_writer = cv2.VideoWriter(output_video_file, fourcc, fps, (video_width, video_height),True)

def count_file_in_folder(folder_path):
    file_list = os.listdir(folder_path)
    
    file_count = 0
    
    for file_name in file_list:
        file_path = os.path.join(folder_path,file_name)
        if os.path.isfile(file_path):
            file_count+=1
            
    return file_count

n = count_file_in_folder(folder_path)

def output():

    for i in range(n):
        image = cv2.imread(folder_path+'fig_{}.png'.format(i))
        # image = image[:,20:-30]
        video_writer.write(image)
    video_writer.release()

    # for i in range(32,160):
    #     image = cv2.imread(folder_path+'3d_{}.png'.format(i))
    #     video_writer.write(image)
    # video_writer.release()


output()
cv2.destroyAllWindows()
