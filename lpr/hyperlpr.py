#coding=utf-8
import cv2
import numpy as np
import os

chars = [u"京", u"沪", u"津", u"渝", u"冀", u"晋", u"蒙", u"辽", u"吉", u"黑", u"苏", u"浙", u"皖", u"闽", u"赣", u"鲁", u"豫", u"鄂", u"湘", u"粤", u"桂",
             u"琼", u"川", u"贵", u"云", u"藏", u"陕", u"甘", u"青", u"宁", u"新", u"0", u"1", u"2", u"3", u"4", u"5", u"6", u"7", u"8", u"9", u"A",
             u"B", u"C", u"D", u"E", u"F", u"G", u"H", u"J", u"K", u"L", u"M", u"N", u"P", u"Q", u"R", u"S", u"T", u"U", u"V", u"W", u"X",
             u"Y", u"Z",u"港",u"学",u"使",u"警",u"澳",u"挂",u"军",u"北",u"南",u"广",u"沈",u"兰",u"成",u"济",u"海",u"民",u"航",u"空"
             ]



class LPR():
    def __init__(self, folder):
        """
        Init the recognition instance.

        :param model_finemapping:    finemapping model which deskew the license plate
        :param model_rec:   CNN based sequence recognition model trained with CTC loss.
        """

        charLocPath = os.path.join(folder, "cascade/char/char_single.xml")
        modelRecognitionPath = [os.path.join(folder, "dnn/SegmenationFree-Inception.prototxt"), os.path.join(folder, "dnn/SegmenationFree-Inception.caffemodel")]
        modelFineMappingPath = [os.path.join(folder, "dnn/HorizonalFinemapping.prototxt"), os.path.join(folder, "dnn/HorizonalFinemapping.caffemodel")]
        self.charLoc = cv2.CascadeClassifier(charLocPath)
        self.modelFineMapping = cv2.dnn.readNetFromCaffe(*modelFineMappingPath)
        self.modelRecognition = cv2.dnn.readNetFromCaffe(*modelRecognitionPath)

    def decodeCTC(self, y_pred):
        # """
        # Decode  the results from the last layer of recognition model.
        # :param y_pred:  the feature map output last feature map.
        # :return: decode results.
        # """
        results = ""
        confidence = 0.0
        y_pred = y_pred.T
        table_pred = y_pred
        res = table_pred.argmax(axis=1)
        for i, one in enumerate(res):
            if one<len(chars) and (i==0 or (one!=res[i-1])):
                results += chars[one]
                confidence += table_pred[i][one]
        confidence /= len(results)
        return results, confidence

    def fitLineRansac(self, pts, zero_add = 0):
        if len(pts) >= 2:
            [vx, vy, x, y] = cv2.fitLine(pts, cv2.DIST_HUBER, 0, 0.01, 0.01)
            lefty = int((-x * vy / vx) + y)
            righty = int(((136 - x) * vy / vx) + y)
            return lefty + 30 + zero_add, righty + 30 + zero_add
        return 0, 0

    def fineMappingOrigin(self, image_rgb):
        line_upper = []
        line_lower = []
        line_experiment = []
        gray_image = cv2.cvtColor(image_rgb, cv2.COLOR_BGR2GRAY)
        for k in np.linspace(-50, 0, 16):
            binary_niblack = cv2.adaptiveThreshold(gray_image, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 17, k)
            imagex, contours, hierarchy = cv2.findContours(binary_niblack.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                bdbox = cv2.boundingRect(contour)
                if ((bdbox[3] / float(bdbox[2]) > 0.7 and bdbox[3] * bdbox[2] > 100 and bdbox[3] * bdbox[2] < 1200) or (
                        bdbox[3] / float(bdbox[2]) > 3 and bdbox[3] * bdbox[2] < 100)):
                    line_upper.append([bdbox[0], bdbox[1]])
                    line_lower.append([bdbox[0] + bdbox[2], bdbox[1] + bdbox[3]])
                    line_experiment.append([bdbox[0], bdbox[1]])
                    line_experiment.append([bdbox[0] + bdbox[2], bdbox[1] + bdbox[3]])
        rgb = cv2.copyMakeBorder(image_rgb, 30, 30, 0, 0, cv2.BORDER_REPLICATE)
        leftyA, rightyA = self.fitLineRansac(np.array(line_lower), 3)
        leftyB, rightyB = self.fitLineRansac(np.array(line_upper), -3)
        rows, cols = rgb.shape[:2]
        pts_map1 = np.float32([[cols - 1, rightyA], [0, leftyA], [cols - 1, rightyB], [0, leftyB]])
        pts_map2 = np.float32([[136, 36], [0, 36], [136, 0], [0, 0]])
        mat = cv2.getPerspectiveTransform(pts_map1, pts_map2)
        image = cv2.warpPerspective(rgb, mat, (136, 36), flags=cv2.INTER_CUBIC)
        return image

    def fineMappingBySelect(self, image_rgb, line_upper, line_lower):
        rgb = cv2.copyMakeBorder(image_rgb, 30, 30, 0, 0, cv2.BORDER_REPLICATE)
        leftyA, rightyA = self.fitLineRansac(np.array(line_lower), 3)
        leftyB, rightyB = self.fitLineRansac(np.array(line_upper), -3)
        rows, cols = rgb.shape[:2]
        pts_map1 = np.float32([[cols - 1, rightyA], [0, leftyA], [cols - 1, rightyB], [0, leftyB]])
        pts_map2 = np.float32([[136, 36], [0, 36], [136, 0], [0, 0]])
        mat = cv2.getPerspectiveTransform(pts_map1, pts_map2)
        image = cv2.warpPerspective(rgb, mat, (136, 36), flags=cv2.INTER_CUBIC)
        return image

    def fineMapping(self, image, charSelection = False):
        image = cv2.resize(image, (204, 54))
        watches = self.charLoc.detectMultiScale(image, 1.08, 1, minSize=(15, 15))
        upper = [[x,y] for x,y,w,h in watches]
        lower = [[x+w,y+h] for x,y,w,h in watches]
        if len(watches) > 3:
            fined = self.fineMappingBySelect(image, upper, lower)
        else:
            if charSelection:
                return None, None
            else:
                fined = self.fineMappingOrigin(image)
        blob = cv2.dnn.blobFromImage(fined.copy(), 1.0 / 255.0, (66, 16), (0,0,0), False, False)
        self.modelFineMapping.setInput(blob)
        X1, X2 = self.modelFineMapping.forward()[0]
        W = fined.shape[1]
        margin = 0.03
        X1 -= margin
        X2 += margin
        X1 = max(0,int(X1*W))
        X2 = min(W,int(X2*W))
        fined = fined[:,X1:X2]
        return fined

    # 车牌字符分段识别
    def segmentationFreeRecognition(self, src):
        """

        :param src:
        :return:
        """
        temp = cv2.resize(src, (160,40))
        # transpose 交换了0轴和1轴的点
        temp = temp.transpose(1, 0, 2)
        blob = cv2.dnn.blobFromImage(temp, 1/255.0, (40,160), (0,0,0), False, False)
        self.modelRecognition.setInput(blob)
        y_pred = self.modelRecognition.forward()[0]
        y_pred = y_pred[:,2:,:]
        y_pred = np.squeeze(y_pred)
        return self.decodeCTC(y_pred)

    # 主函数
    def plateRecognition(self, image, charSelectionDeskew = True):
        """
        the simple pipline consists of detection . deskew , fine mapping alignment, recognition.
        :param image: the input BGR image from imread used by opencv
        :param charSelectionDeskew: use character detection when fine mapping stage which will reduce the False Accept Rate as far as possible.
        :return: will return [ [plate1 string ,confidence1, location1  ],
                               [plate2 string ,confidence2, location2  ] ....
                            ]

        """
        res_set = []
        plate  = image
        image_rgb = self.fineMapping(plate, charSelectionDeskew)
        if image_rgb is not None:
            res, confidence = self.segmentationFreeRecognition(image_rgb)
            res_set.append([res, confidence])
        return res_set
