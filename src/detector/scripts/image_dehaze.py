#! /usr/bin/env python3

import cv2
import math
import numpy as np

class ImageDehze():
    def __init__(self):
        self.kernel_size = 15

    def DarkChannel(self, cv_image64):
        b, g, r = cv2.split(cv_image64)
        dc = cv2.min(cv2.min(r, g), b)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (self.kernel_size, self.kernel_size))
        dark = cv2.erode(dc, kernel)

        return dark

    def AtmLight(self, cv_image64):
        dark = self.DarkChannel(cv_image64)
        [h, w] = cv_image64.shape[:2]
        imsz = h * w
        numpx = int(max(math.floor(imsz/1000), 1))
        darkvec = dark.reshape(imsz)
        imvec = cv_image64.reshape(imsz,3)

        indices = darkvec.argsort()
        indices = indices[imsz-numpx::]

        atmsum = np.zeros([1,3])
        for ind in range(1,numpx):
            atmsum = atmsum + imvec[indices[ind]]

        A = atmsum / numpx

        return A

    def TransmissionEstimate(self, cv_image64):
        A = self.AtmLight(cv_image64)
        omega = 0.95
        im3 = np.empty(cv_image64.shape,cv_image64.dtype)

        for ind in range(0,3):
            im3[:,:,ind] = cv_image64[:,:,ind]/A[0,ind]
        dark = self.DarkChannel(im3)
        transmission = 1 - omega * dark

        return transmission

    def Guidedfilter(self, cv_image, p, r, eps):
        mean_I = cv2.boxFilter(cv_image, cv2.CV_64F,(r,r))
        mean_p = cv2.boxFilter(p, cv2.CV_64F,(r,r))
        mean_Ip = cv2.boxFilter(cv_image*p,cv2.CV_64F,(r,r))
        cov_Ip = mean_Ip - mean_I*mean_p

        mean_II = cv2.boxFilter(cv_image*cv_image, cv2.CV_64F,(r,r))
        var_I   = mean_II - mean_I*mean_I

        a = cov_Ip/(var_I + eps)
        b = mean_p - a*mean_I

        mean_a = cv2.boxFilter(a,cv2.CV_64F,(r,r))
        mean_b = cv2.boxFilter(b,cv2.CV_64F,(r,r))

        q = mean_a*cv_image + mean_b

        return q

    def TransmissionRefine(self, cv_image, et):
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        gray = np.float64(gray)/255
        r = 60
        eps = 0.0001
        t = self.Guidedfilter(gray,et,r,eps)

        return t

    def Recover(self, cv_image64, t, tx = 0.1):
        A = self.AtmLight(cv_image64)
        res = np.empty(cv_image64.shape, cv_image64.dtype)
        t = cv2.max(t,tx)

        for ind in range(0,3):
            res[:,:,ind] = (cv_image64[:,:,ind]-A[0,ind])/t + A[0,ind]

        return res
    
    def get_image_dehaze(self, cv_image):
        I = cv_image.astype('float64')/255
        te = self.TransmissionEstimate(I)
        t = self.TransmissionRefine(cv_image, te)
        J = self.Recover(I,t,0.1)
        raw64 = J * 255
        raw8 = np.maximum(np.minimum(raw64, 255), 0).astype(np.uint8)
        return raw8
