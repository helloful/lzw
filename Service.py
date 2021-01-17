#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
from std_srvs.srv import SetBool, SetBoolResponse
from ctypes import *
import time
import wave
import ctypes
from playsound import playsound
FRAME_LEN = 640  # Byte
MSP_SUCCESS = 0
# 返回结果状态
MSP_AUDIO_SAMPLE_FIRST = 1
MSP_AUDIO_SAMPLE_CONTINUE = 2
MSP_AUDIO_SAMPLE_LAST = 4
MSP_REC_STATUS_COMPLETE = 5

poetry={'静夜思':"床前明月光，疑是地上霜。举头望明月，低头思故乡。",
        '早发白帝城':"朝辞白帝彩云间，千里江陵一日还。两岸猿声啼不住，轻舟已过万重山。"}

def getpoetry():
    global poetry
    f = open("tmp.txt", "r")
    a = f.read()
    f.close()
    if len(a) > 0:
        poetry = eval(a)

# 调用动态链接库
dll = cdll.LoadLibrary("/usr/local/src/python_code/Linux_aisound_exp1227_5f0fc56a/libs/x64/libmsc.so")
login_params = b"appid = 5f0fc56a, work_dir = Linux_aisound_exp1227_5f0fc56a"

class Msp:
    def __init__(self):
        pass
    def login(self):
        ret = dll.MSPLogin(None, None, login_params)
    def logout(self):
        ret = dll.MSPLogout()
    def iat(self, text):
        session_begin_params='engine_type = local, voice_name = xiaoyan, text_encoding = gbk, speed_increase = 1, effect = 0, tts_res_path  = fo|Linux_aisound_exp1227_5fddabe1\\bin\\msc\\res\\tts\\xiaoyan.jet;fo|Linux_aisound_exp1227_5fddabe1\\bin\\msc\\res\\tts\\common.jet'
        ret = c_int(0)
        dll.QTTSSessionBegin.restype = c_char_p
        sessionID=dll.QTTSSessionBegin(session_begin_params,byref(ret))
        # print(text.encode())
        ret=dll.QTTSTextPut(sessionID, text,len(text),None)
        mfile=wave.open(r"output.wav","wb")
        mfile.setnchannels(1)
        mfile.setsampwidth(2)
        mfile.setframerate(16000)
        audio_len=c_uint64()
        synth_status=c_uint64()
        getret=c_uint64()
        # ret = c_int()
        if ret==0:
            while True:
                dll.QTTSAudioGet.restype = c_uint64

                data=dll.QTTSAudioGet(sessionID,byref(audio_len),byref(synth_status),byref(getret))

                #print(audio_len.value)
                if data:
                    # print(type(audio_len.value))
                    wd=string_at(data,audio_len.value  )
                    mfile.writeframes(wd)
                if synth_status.value == 2:
                    break
                # time.sleep(0.1)
        mfile.close()

        dll.QTTSSessionEnd(sessionID,'normal end')
        playsound('output.wav')

def XF_text(text):
    msp = Msp()
    msp.login()

    msp.iat(text)
    msp.logout()

def string_callback(req):
    getpoetry()
    if req.data:
        ttt=rospy.get_param('truth').replace('。','')
        print(ttt)
        print(poetry[ttt])
        #text = "床前明月光，疑是地上霜。举头望明月，低头思故乡。".encode('gbk')
        XF_text(poetry[ttt].encode('gbk'))
        return SetBoolResponse(True, "Print Successully")
    else:
        return SetBoolResponse(False, "Print Failed")

def string_server():
    rospy.init_node('string_server')
    s = rospy.Service('print_string',SetBool, string_callback)
    print('Ready to print hello string.')
    rospy.spin()

if __name__ == '__main__':
    string_server()
