import threading
import rospy
from std_srvs.srv import SetBool, SetBoolRequest
from ctypes import *
import time
import pyaudio
import wave
import tkinter as tk
FRAME_LEN = 640  # Byte
MSP_SUCCESS = 0
# 返回结果状态
MSP_AUDIO_SAMPLE_FIRST = 1
MSP_AUDIO_SAMPLE_CONTINUE = 2
MSP_AUDIO_SAMPLE_LAST = 4
MSP_REC_STATUS_COMPLETE = 5
dll = cdll.LoadLibrary(r"/usr/local/src/python_code/Linux_aitalk_exp1227_5f0fc56a/libs/x64/libmsc.so")
in_path= "input1.wav"
login_params = b"appid = 5f0fc56a, work_dir = Linux_aitalk_exp1227_5f0fc56a"
class Msp:
    def __init__(self):
        pass
    def login(self):
        ret = dll.MSPLogin(None, None, login_params)

    def logout(self):
        ret = dll.MSPLogout()

    def isr(self, audiofile, session_begin_params):
        ret = c_int()
        sessionID = c_voidp()
        dll.QISRSessionBegin.restype = c_char_p
        sessionID = dll.QISRSessionBegin(None, session_begin_params, byref(ret))

        piceLne = 1638 * 2
        epStatus = c_int(0)
        recogStatus = c_int(0)

        wavFile = open(audiofile, 'rb')
        wavData = wavFile.read(piceLne)

        ret = dll.QISRAudioWrite(sessionID, wavData, len(wavData), MSP_AUDIO_SAMPLE_FIRST, byref(epStatus),
                                 byref(recogStatus))
        time.sleep(0.1)
        while wavData:
            wavData = wavFile.read(piceLne)

            if len(wavData) == 0:
                break
            ret = dll.QISRAudioWrite(sessionID, wavData, len(wavData), MSP_AUDIO_SAMPLE_CONTINUE, byref(epStatus),
                                     byref(recogStatus))
            time.sleep(0.1)
        wavFile.close()
        ret = dll.QISRAudioWrite(sessionID, None, 0, MSP_AUDIO_SAMPLE_LAST, byref(epStatus), byref(recogStatus))
        laststr = ''
        counter = 0
        while recogStatus.value != MSP_REC_STATUS_COMPLETE:
            print(recogStatus.value)
            ret = c_int()
            dll.QISRGetResult.restype = c_char_p
            retstr = dll.QISRGetResult(sessionID, byref(recogStatus), 0, byref(ret))
            if retstr is not None:
                laststr += retstr.decode()
            counter += 1
            time.sleep(0.2)
            counter += 1

        return laststr
def get_audio(filepath):
    aa = str(input("是否开始录音？   （1是/0否）"))
    if aa == str("1") :
        CHUNK = 256
        FORMAT = pyaudio.paInt16
        CHANNELS = 1                # 声道数
        RATE = 11025                # 采样率
        RECORD_SECONDS = 5
        WAVE_OUTPUT_FILENAME = filepath
        p = pyaudio.PyAudio()

        stream = p.open(format=FORMAT,
                        channels=CHANNELS,
                        rate=RATE,
                        input=True,
                        frames_per_buffer=CHUNK)

        print("*"*10, "开始录音：请在5秒内输入语音")
        frames = []
        for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
            data = stream.read(CHUNK)
            frames.append(data)
        print("*"*10, "录音结束\n")

        stream.stop_stream()
        stream.close()
        p.terminate()

        wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames))
        wf.close()
    elif aa == str("否"):
        exit()
    else:
        print("无效输入，请重新选择")
        get_audio(in_path)

def XF_text(filepath, audio_rate):
    msp = Msp()
    #print("登录科大讯飞")
    msp.login()
    #print("科大讯飞登录成功")
    session_begin_params = b"sub = iat, ptt = 0, result_encoding = utf8, result_type = plain, domain = iat"
    if 16000 == audio_rate:
        session_begin_params = b"sub = iat, domain = iat, language = zh_cn, accent = mandarin, sample_rate = 16000, result_type = plain, result_encoding = utf8"
    text = msp.isr(filepath, session_begin_params)
    msp.logout()
    return text
def string_client(text):
    rospy.init_node('string_client')
    rospy.wait_for_service('print_string')
    rospy.set_param('truth', text)
    string_client = rospy.ServiceProxy('print_string',SetBool)
    response = string_client(True)
    return response.success, response.message
def gameStart():
    print("游戏开始")
def gameOver():
    print("游戏结束")
class Client():
    def __init__(self):
        self.initThread()
    def initThread(self):
        self.guiThread=threading.Thread(target=self.initGUI)
        self.AudioThread=threading.Thread(target=self.audioTetx)
    def audioText(self):
        while True:
            get_audio(in_path)
            text = XF_text(in_path, 16000)
            string_client(text)
    def initGUI(self):
        window = tk.Tk()
        window.title("猜数小游戏服务端")
        window.geometry('500x250')
        strShow1 = tk.StringVar()
        label1 = tk.Label(window, textvariable=strShow1, font=("楷体", 18))
        label1.place(x=150, y=20)

        strShow3 = tk.StringVar()
        label3 = tk.Label(window, textvariable=strShow3, font=("楷体", 18), fg="red")
        label3.place(x=220, y=70)

        button = tk.Button(window, text="开始", command=gameStart, width=18)
        button.place(x=180, y=120)

        button1 = tk.Button(window, text="退出", command=gameOver, width=18)
        button1.place(x=180, y=160)

        strShow2 = tk.StringVar()
        label2 = tk.Label(window, textvariable=strShow2, font=("楷体", 12), fg="red")
        label2.place(x=180, y=180)

        window.mainloop()


    def start(self):
        self.guiThread.start()
        self.AudioThread.start()


if __name__ == '__main__':
    client=Client()
    client.start()