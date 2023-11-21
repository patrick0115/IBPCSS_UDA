from pandas import read_csv
import numpy as np
def get_palette():
    df = read_csv('/home/icalab/UDA_PCSS/src/IBPCSS_UDA/seg/tools/color_coding_semantic_segmentation_classes - Sheet1.csv',)
    data = df.values
    
    palette = np.random.randint(0, 150, (150, 3), dtype=np.uint8)
    row_data_list = []
    # print(palette.shape)
    for i in range(150):
        palette[i][0]=int(data[i][5][1:-1].split(",")[2])
        palette[i][1]=int(data[i][5][1:-1].split(",")[1])
        palette[i][2]=int(data[i][5][1:-1].split(",")[0])
        # print(palette[i])
    row_data = data[:, 8]
    for item in row_data:
        row_data_list.append(str(item))
    return palette, row_data_list


