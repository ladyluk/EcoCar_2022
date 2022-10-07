import rospy
from GT_CAVs_ROS_Messages.msg import PcmToCav5, PcmToCav3
import os
import sys
from PyQt5 import QtWidgets
from PyQt5.QtCore import Qt, pyqtSignal, QThread
from PyQt5.QtWidgets import*
from PyQt5.QtGui import*
from PyQt5 import QtCore, QtMultimedia
from PyQt5.QtMultimedia import QMediaPlayer, QMediaContent
from PyQt5.QtCore import QTimer

from std_msgs.msg import Float32
import rospy
import time

displayButton = False

#voicefolder
voice_folder = "ACCSoundFiles"
voice_file = ["Exit Prompt.mp3", "ACC Prompt.mp3", "Welcome Screen.mp3", "Tutorial Touchscreen Navi.mp3"]

# DETELE AFTER TESTING!!!
#for i in range(59):
#    voice_file.append("6-sec.wav")

incorrect_sound = "ACCSoundFiles/incorrect2.mp3"
correct_sound = "ACCSoundFiles/correct3.wav"
do_sound_effect = True

#pages
slides_folder = 'ACCSlides/V1 Figma Y4 Prototype Revision-images'
slides_file_list = []
for filename in os.listdir(slides_folder):
    f = os.path.join(slides_folder, filename)
    slides_file_list.append (f)

slides_file_list.sort()
page_list = [
            'ExitPrompt',
            'ACCPrompt',
            'WelcomeScreen',
            'TutorialTouchscreenNavi', 'TutorialCtrlNavi', 'TutorialCtrlNavi2',
            'GoalPrompt1', 'GoalPrompt2','GoalPrompt3','GoalPrompt4','GoalPrompt5','GoalPrompt6','GoalPrompt7',
            'IntroductionTitle','Introduction1', 'Introduction2', 'Introduction3',
            'UsageCondition1', 'UsageCondition2', 'UsageCondition3', 'UsageCondition4',
            'EngagingACCTitle', 'EngagingACC1', 'EngagingACC2', 'EngagingACC3',
            'AdjustingSpeedTitle', 'AdjustingSpeed1', 'AdjustingSpeed2','AdjustingSpeed3', 'AdjustingSpeed4',
            'Try1ReviewTitle', 'Try1Review1', 'Try1Review2', 'TryEngageAdjust',  
            'DisengagingACCTitle','DisengagingACC1', 'DisengagingACC2','DisengagingACC3', 'DisengagingACC4','DisengagingACC5','DisengagingACC6','DisengagingACC7',
            'ResumingACCTitle', 'ResumingACC1','ResumingACC2','ResumingACC3','ResumingACC4',
            'Try2ReviewTitle', 'Try2Review1', 'Try2Review2', 'TryDisengageAdjust',
            'SettingGapDistanceTitle', 'SettingGapDistance1', 'SettingGapDistance2', 'SettingGapDistance3',
            'SwitchingfromACCtoCCTitle','SwitchingfromACCtoCC1','SwitchingfromACCtoCC2',
            'Try3ReviewTitle', 'Try3Review','TryGapSwitch', 
            'TableofContents'
            ]

# button status
# 6: "Cancel" 
# 5: "Cruise On/Off"
# 4: "SL/SW On/Off/GAP" collision
# 3: "Set" thumbwheel
# 2: "Resume" thumbwheel
# 1: no button pressed
# accel, brake
isNoButtonPressed = True
isCancelPressed = False
isCruiseOnOffPressed = False
isGapSwPressed = False
gapSwCount = 0
isSetPressed = False
isResumePressed = False
isAccelPedalPressed = False
isBrakePedalPressed = False
timerTwoSec = QTimer(interval=1500)
timer1p5Sec = QTimer(interval=1500)
prevCrsSpdLmtrSwStat = 1
prevGapSwAct = 0

default_flash_times = 3

# flash button
# example:
# button = flashButton(self.page2)
# button.flash(3) # will make it flash for 3 seconds
class flashButton(QPushButton):
    def __init__(self, input):
        super().__init__(input)
        self.count = 0
        self.totalCount = 0
        self.timer = QTimer(self, interval=100)
        self.timer.timeout.connect(self.flashing)

        
    def flash(self, numOfFlashes):
        self.setStyleSheet('border: 0px')
        if(self.timer.isActive()):
            self.timer.stop()
        self.count = 0
        self.totalCount = numOfFlashes*10
        self.timer.start()

    def flash2(self, event):
        self.setStyleSheet('border: 0px')
        if(self.timer.isActive()):
            self.timer.stop()
        self.count = 0
        self.totalCount = 30
        self.timer.start()

    def flashing(self):
        if self.count%10==1 or self.count%10==4:
            self.setStyleSheet('border: 5px solid #29e671; border-radius: 20%')
        elif self.count%10==3 or self.count%10==6:
            self.setStyleSheet('border: 0px')
        if(self.count==self.totalCount):
            self.timer.stop()
            self.setStyleSheet('border: 0px')
        else:
            self.count+=1

    


class widget(QWidget):
    def __init__(self, parent = None):
        super().__init__(parent)
        self.setWindowTitle("ACC Control Tutorial")
        self.resize(1920, 1080)

        self.next_target = None

        #finding centerpoint
        qtRectangle = self.frameGeometry() #retrieving geometry of the window
        centerpoint = QDesktopWidget().availableGeometry().center() #obtains center of screen
        #moves window to center of screen
        qtRectangle.moveCenter(centerpoint) 
        self.move(qtRectangle.topLeft())
        
        self.setclicked = False
        self.resumeclicked = False

        #create stacked layout
        self.stack = QStackedWidget(self)
        self.exitprompt = QWidget() 
        self.accprompt = QWidget()
        self.welcomescreen = QWidget()
        self.tutorialtouchscreennavi = QWidget() 
        self.tutorialctrlnavi = QWidget()
        self.tutorialctrlnavi2 = QWidget()
        self.goalprompt1 = QWidget()
        self.goalprompt2 = QWidget() 
        self.goalprompt3 = QWidget()
        self.goalprompt4 = QWidget()
        self.goalprompt5 = QWidget() 
        self.goalprompt6 = QWidget() 
        self.goalprompt7 = QWidget()
        self.introductiontitle = QWidget()
        self.introduction1 = QWidget()
        self.introduction2 = QWidget()
        self.introduction3 = QWidget()
        self.usagecondition1 = QWidget()
        self.usagecondition2 = QWidget()
        self.usagecondition3 = QWidget()
        self.usagecondition4 = QWidget()
        self.engagingtitle = QWidget()
        self.engagingacc1 = QWidget()
        self.engagingacc2 = QWidget()
        self.engagingacc3 = QWidget()
        self.adjustingspeedtitle = QWidget()
        self.adjustingspeed1 = QWidget()
        self.adjustingspeed2 = QWidget()
        self.adjustingspeed3 = QWidget()
        self.adjustingspeed4 = QWidget()
        self.try1reviewtitle = QWidget()
        self.try1review1 = QWidget()
        self.try1review2 = QWidget()
        self.tryengageadjust = QWidget()
        self.disengagingacctitle = QWidget()
        self.disengagingacc1 = QWidget()
        self.disengagingacc2 = QWidget()
        self.disengagingacc3 = QWidget()
        self.disengagingacc4 = QWidget()
        self.disengagingacc5 = QWidget()
        self.disengagingacc6 = QWidget()
        self.disengagingacc7 = QWidget()
        self.resumingacctitle = QWidget()
        self.resumingacc1 = QWidget()
        self.resumingacc2 = QWidget()
        self.resumingacc3 = QWidget()
        self.resumingacc4 = QWidget()
        self.try2reviewtitle = QWidget()
        self.try2review1 = QWidget()
        self.try2review2 = QWidget()
        self.trydisengageadjust = QWidget()
        self.settinggapdistancetitle = QWidget()
        self.settinggapdistance1 = QWidget()
        self.settinggapdistance2 = QWidget()
        self.settinggapdistance3 = QWidget()
        self.switchingfromacctocctitle = QWidget()
        self.switchingfromacctocc1 = QWidget()
        self.switchingfromacctocc2 = QWidget()
        self.try3reviewtitle = QWidget()
        self.try3review = QWidget()
        self.trygapswitch = QWidget()
        self.tableofcontents = QWidget()                          
        
        self.ExitPrompt()
        self.ACCPrompt()
        self.WelcomeScreen()
        self.TutorialTouchscreenNavi()
        self.TutorialCtrlNavi()
        self.TutorialCtrlNavi2()
        self.GoalPrompt1()
        self.GoalPrompt2()
        self.GoalPrompt3()
        self.GoalPrompt4()
        self.GoalPrompt5()
        self.GoalPrompt6()
        self.GoalPrompt7()
        self.IntroductionTitle()
        self.Introduction1()
        self.Introduction2()
        self.Introduction3()
        self.UsageCondition1()
        self.UsageCondition2()
        self.UsageCondition3()
        self.UsageCondition4()
        self.EngagingACCTitle()
        self.EngagingACC1()
        self.EngagingACC2()
        self.EngagingACC3()
        self.AdjustingSpeedTitle()
        self.AdjustingSpeed1()
        self.AdjustingSpeed2()
        self.AdjustingSpeed3()
        self.AdjustingSpeed4()
        self.Try1ReviewTitle()
        self.Try1Review1()
        self.Try1Review2()
        self.TryEngageAdjust()
        self.DisengagingACCTitle()
        self.DisengagingACC1()
        self.DisengagingACC2()
        self.DisengagingACC3()
        self.DisengagingACC4()
        self.DisengagingACC5()
        self.DisengagingACC6()
        self.DisengagingACC7()
        self.ResumingACCTitle()
        self.ResumingACC1()
        self.ResumingACC2()
        self.ResumingACC3()
        self.ResumingACC4()
        self.Try2ReviewTitle()
        self.Try2Review1()
        self.Try2Review2()
        self.TryDisengageAdjust()
        self.SettingGapDistanceTitle()
        self.SettingGapDistance1()
        self.SettingGapDistance2()
        self.SettingGapDistance3()
        self.SwitchingfromACCtoCCTitle()
        self.SwitchingfromACCtoCC1()
        self.SwitchingfromACCtoCC2()
        self.Try3ReviewTitle()
        self.Try3Review()
        self.TryGapSwitch()
        self.TableofContents()
        
        self.stack.addWidget(self.exitprompt)
        self.stack.addWidget(self.accprompt)
        self.stack.addWidget(self.welcomescreen)
        self.stack.addWidget(self.tutorialtouchscreennavi)
        self.stack.addWidget(self.tutorialctrlnavi)
        self.stack.addWidget(self.tutorialctrlnavi2)
        self.stack.addWidget(self.goalprompt1)
        self.stack.addWidget(self.goalprompt2)
        self.stack.addWidget(self.goalprompt3)
        self.stack.addWidget(self.goalprompt4)
        self.stack.addWidget(self.goalprompt5)
        self.stack.addWidget(self.goalprompt6)
        self.stack.addWidget(self.goalprompt7)
        self.stack.addWidget(self.introductiontitle)
        self.stack.addWidget(self.introduction1)
        self.stack.addWidget(self.introduction2)
        self.stack.addWidget(self.introduction3)
        self.stack.addWidget(self.usagecondition1)
        self.stack.addWidget(self.usagecondition2)
        self.stack.addWidget(self.usagecondition3)
        self.stack.addWidget(self.usagecondition4)
        self.stack.addWidget(self.engagingtitle)
        self.stack.addWidget(self.engagingacc1)
        self.stack.addWidget(self.engagingacc2)
        self.stack.addWidget(self.engagingacc3)
        self.stack.addWidget(self.adjustingspeedtitle)
        self.stack.addWidget(self.adjustingspeed1)
        self.stack.addWidget(self.adjustingspeed2)
        self.stack.addWidget(self.adjustingspeed3)
        self.stack.addWidget(self.adjustingspeed4)
        self.stack.addWidget(self.try1reviewtitle)
        self.stack.addWidget(self.try1review1)
        self.stack.addWidget(self.try1review2)
        self.stack.addWidget(self.tryengageadjust)
        self.stack.addWidget(self.disengagingacctitle)
        self.stack.addWidget(self.disengagingacc1)
        self.stack.addWidget(self.disengagingacc2)
        self.stack.addWidget(self.disengagingacc3)
        self.stack.addWidget(self.disengagingacc4)
        self.stack.addWidget(self.disengagingacc5)
        self.stack.addWidget(self.disengagingacc6)
        self.stack.addWidget(self.disengagingacc7)
        self.stack.addWidget(self.resumingacctitle)
        self.stack.addWidget(self.resumingacc1)
        self.stack.addWidget(self.resumingacc2)
        self.stack.addWidget(self.resumingacc3)
        self.stack.addWidget(self.resumingacc4)
        self.stack.addWidget(self.try2reviewtitle)
        self.stack.addWidget(self.try2review1)
        self.stack.addWidget(self.try2review2)
        self.stack.addWidget(self.trydisengageadjust)
        self.stack.addWidget(self.settinggapdistancetitle)
        self.stack.addWidget(self.settinggapdistance1)
        self.stack.addWidget(self.settinggapdistance2)
        self.stack.addWidget(self.settinggapdistance3)
        self.stack.addWidget(self.switchingfromacctocctitle)
        self.stack.addWidget(self.switchingfromacctocc1)
        self.stack.addWidget(self.switchingfromacctocc2)
        self.stack.addWidget(self.try3reviewtitle)
        self.stack.addWidget(self.try3review)
        self.stack.addWidget(self.trygapswitch)
        self.stack.addWidget(self.tableofcontents)

        
        self.stack.setCurrentIndex(1)
        index = self.stack.currentIndex()

        pcm_page_number_pub.publish(125)
        time.sleep(1)

        # send slide number
        send_num = 0
        if self.stack.currentIndex() >= 13 and self.stack.currentIndex() <= 58:
            send_num = self.stack.currentIndex() - 13
        else:
            send_num = 0
        pcm_page_number_pub.publish(send_num)
        
        self.previouspage = 1

        self.player = QtMultimedia.QMediaPlayer()
        self.player.mediaStatusChanged[QMediaPlayer.MediaStatus].connect(self.playWhenReady)
        self.needToPlayVoiceover = False

        # play voiceover for first page if necessary
        self.displayVoiceover()

        self.stack.currentChanged.connect(lambda: self.changePageCallback())
        self.showFullScreen()
        # self.show() 

    #display picture
    def displaypic(self, index):    
        #index = self.currentPage()
        pic_dir = slides_file_list[index]
        image = QPixmap(pic_dir)
        return image

    def changePageCallback(self):
        global isCancelPressed
        global isCruiseOnOffPressed
        global isGapSwPressed
        global gapSwCount
        global isSetPressed
        global isResumePressed
        global isAccelPedalPressed
        global isBrakePedalPressed
        global prevCrsSpdLmtrSwStat
        global prevGapSwAct
        # display voiceover
        self.displayVoiceover()
        # initialize button state
        isCancelPressed = False
        isCruiseOnOffPressed = False
        isGapSwPressed = False
        gapSwCount = 0
        isSetPressed = False
        isResumePressed = False
        isAccelPedalPressed = False
        isBrakePedalPressed = False
        prevCrsSpdLmtrSwStat = 1
        prevGapSwAct = 0
        self.next_target = None
        # send slide number
        send_num = 0
        if self.stack.currentIndex() >= 13 and self.stack.currentIndex() <= 61:
            if self.stack.currentIndex() >= 13 and self.stack.currentIndex() <= 30:
                send_num = self.stack.currentIndex() - 13
            elif self.stack.currentIndex() >= 31 and self.stack.currentIndex() <= 47:
                send_num = self.stack.currentIndex() - 14
            elif self.stack.currentIndex() >= 48 and self.stack.currentIndex() <= 58:
                send_num = self.stack.currentIndex() - 15
            else:
                send_num = self.stack.currentIndex() - 16
        else:
            send_num = 0

        # print(send_num)
        pcm_page_number_pub.publish(send_num)
        

    #decide which voiceover to play based on page
    def displayVoiceover(self):
        if(self.player.state()==QMediaPlayer.PlayingState):
            # previous sound is playing, need to stop it first
            self.player.stop()
        if(self.stack.currentIndex()<len(voice_file)):
            self.voiceover(voice_file[self.stack.currentIndex()])
        else:
            print("displayVoiceover: voice file out of index")
            

    #returns current page index
    def currentPage(self):
        page = self.stack.currentIndex()
        return page        

    #mouse-click: self.stack.setCurrentIndex(next page)
    def next_page(self, event):
        index = self.stack.currentIndex() + 1
        if index < self.stack.count():
            self.previouspage = self.currentPage()
            self.stack.setCurrentIndex(index)
            #print("Mouse Click")
    #self.stack.setCurrentIndex(next page)
    def gotonext(self):
        index = self.stack.currentIndex() + 1
        if index < self.stack.count():
            self.previouspage = self.currentPage()
            self.stack.setCurrentIndex(index)
    def gotonext_timer(self):
        timerTwoSec.stop()
        index = self.stack.currentIndex() + 1
        if index < self.stack.count():
            self.previouspage = self.currentPage()
            self.stack.setCurrentIndex(index)
    def gotonext_timer1p5(self):
        global isCancelPressed
        timer1p5Sec.stop()
        isCancelPressed = False
        self.play_warning(False)
        rt.adjustingspeed1_next_pg_signal.emit()
        # index = self.stack.currentIndex() + 1
        # if index < self.stack.count():
        #     self.previouspage = self.currentPage()
        #     self.stack.setCurrentIndex(index)
    #self.stack.setCurrentIndex(previous page)
    def gotoprevious(self):
        index = self.stack.currentIndex() - 1
        if index > -1:
            self.previouspage = self.currentPage()
            self.stack.setCurrentIndex(index)
            
    def gotopage(self, page):    
        self.previouspage = self.currentPage()
        self.stack.setCurrentIndex(page)

    def voiceover(self, voice_file):
        if(self.player.state()==QMediaPlayer.PlayingState):
            # previous sound is playing, need to stop it first
            self.player.stop()
        filename = os.path.join(voice_folder, voice_file)
        url = QtCore.QUrl.fromLocalFile(os.path.abspath(filename))
        self.needToPlayVoiceover = True
        self.player.setMedia(QtMultimedia.QMediaContent(url))
    
    def play_warning(self, is_warning = True):
        if not do_sound_effect:
            return
        if(self.player.state()==QMediaPlayer.PlayingState):
            # previous sound is playing, need to stop it first
            self.player.stop()
        if is_warning:
            filename = os.path.join(incorrect_sound)
        else:
            filename = os.path.join(correct_sound)
        url = QtCore.QUrl.fromLocalFile(os.path.abspath(filename))
        self.needToPlayVoiceover = True
        self.player.setMedia(QtMultimedia.QMediaContent(url))
        
    def playWhenReady(self, status):
        if(self.needToPlayVoiceover and status==QtMultimedia.QMediaPlayer.LoadedMedia):
            self.needToPlayVoiceover = False
            self.player.play()
            return
    
    def back_arrow(self, back_arrow):
        back_arrow.resize(200,200)
        back_arrow.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        back_arrow.move(30,30)
        back_arrow.clicked.connect(lambda: self.gotoprevious())
        return back_arrow
    def fwd_arrow(self, fwd_arrow):
        fwd_arrow.resize(200,200)
        fwd_arrow.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        fwd_arrow.move(1700,30)
        fwd_arrow.clicked.connect(lambda: self.gotonext())
        return fwd_arrow
    
    def exit_box(self, exit_box):
        exit_box.resize(120,60)
        exit_box.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        exit_box.move(35,1000)
        exit_box.clicked.connect(lambda: self.gotopage(page_list.index('ExitPrompt')))
        return exit_box

    def ros_button_callback_wrapper(self, btn_num, pressed):
        class btn_fake_msg:
            CrsSpdLmtrSwStat = 1
            def __init__(self, val):
                self.CrsSpdLmtrSwStat = val
        # 6: "Cancel" 
        # 5: "Cruise On/Off"
        # 4: "SL/SW On/Off/GAP" collision
        # 3: "Set" thumbwheel
        # 2: "Resume" thumbwheel
        # 1: placeholder
        if btn_num == 4:
            msg = Float32()
            if pressed:
                msg.data = 1
            else:
                msg.data = 0
            gap_sw_callback_fcn(msg)
        elif btn_num == 2 or btn_num == 3 or btn_num == 5 or btn_num == 6:
            msg = btn_fake_msg(btn_num) if pressed else btn_fake_msg(1)
            callback_fcn_5(msg)

    def ros_brake_callback_wrapper(self, pressed):
        class brake_fake_msg:
            BrkPdlPos = 0
            def __init__(self, val):
                self.BrkPdlPos = val
        if pressed:
            msg = brake_fake_msg(100)
        else:
            msg = brake_fake_msg(0)
        callback_fcn_3(msg)

    def adjustingspeed1flash(self, event):
        page = self.adjustingspeed1
        page.findChild(flashButton, 'set').flash(default_flash_times)
        page.findChild(flashButton, 'resume').flash(default_flash_times)
    
    def adjustingspeed4flash(self, event):
        page = self.adjustingspeed4
        page.findChild(flashButton, 'brake').flash(default_flash_times)
        page.findChild(flashButton, 'thumbwheel').flash(default_flash_times)
    
    def tryengageadjustflash(self, event):
        page = self.tryengageadjust
        page.findChild(flashButton, 'Cancel').flash(default_flash_times)
        page.findChild(flashButton, 'Cruise').flash(default_flash_times)
        page.findChild(flashButton, 'collision').flash(default_flash_times)
        page.findChild(flashButton, 'set').flash(default_flash_times)
        page.findChild(flashButton, 'Resume').flash(default_flash_times)
    
    def trydisengageadjustflash(self, event):
        page = self.trydisengageadjust
        page.findChild(flashButton, 'Cancel').flash(default_flash_times)
        page.findChild(flashButton, 'Cruise').flash(default_flash_times)
        page.findChild(flashButton, 'collision').flash(default_flash_times)
        page.findChild(flashButton, 'set').flash(default_flash_times)
        page.findChild(flashButton, 'Resume').flash(default_flash_times)

    def trygapswitchflash(self, event):
        page = self.trygapswitch
        page.findChild(flashButton, 'Cancel').flash(default_flash_times)
        page.findChild(flashButton, 'Cruise').flash(default_flash_times)
        page.findChild(flashButton, 'collision').flash(default_flash_times)
        page.findChild(flashButton, 'set').flash(default_flash_times)
        page.findChild(flashButton, 'Resume').flash(default_flash_times)

    def exit_program(self):
        pcm_page_number_pub.publish(150)
        self.close()

#ExitPrompt
    def ExitPrompt(self):
        
        #setimage
        index = page_list.index('ExitPrompt')
        page = self.exitprompt

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

        #yes_rectangle
        yes_button = QPushButton(page)
        yes_button.resize(400,150)
        yes_button.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        yes_button.move(450,760)
        yes_button.clicked.connect(lambda: self.exit_program())

        #no_rectangle
        no_button = QPushButton(page)
        no_button.resize(400,150)
        no_button.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        no_button.move(1085,760)
        no_button.clicked.connect(lambda: self.gotopage(self.previouspage))

        page.setAutoFillBackground(True)
        page.setPalette(palette) 

#ACCPrompt
    def ACCPrompt(self):
        
        #setimage
        index = page_list.index('ACCPrompt')
        page = self.accprompt

        image = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        layout = QGridLayout()
        label = QLabel(self)
        label.setPixmap(image)
        layout.addWidget(label)
        page.setLayout(layout)

    #yes_rectangle
        yes_button = QPushButton(page)
        yes_button.resize(400,150)
        yes_button.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        yes_button.move(450,750)
        yes_button.clicked.connect(lambda: self.gotonext())

        #no_rectangle
        no_button = QPushButton(page)
        no_button.resize(400,150)
        no_button.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        no_button.move(1085,750)
        no_button.clicked.connect(lambda: self.exit_program())
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

#WelcomeScreen
    def WelcomeScreen(self):
        
        #setimage
        index = page_list.index('WelcomeScreen')
        page = self.welcomescreen

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#TutorialTouchscreenNavi
    def TutorialTouchscreenNavi(self):
        
        #setimage
        index = page_list.index('TutorialTouchscreenNavi')
        page = self.tutorialtouchscreennavi

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#TutorialCtrlNavi
    def TutorialCtrlNavi(self):
        
        #setimage
        index = page_list.index('TutorialCtrlNavi')
        page = self.tutorialctrlnavi

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #control_pad_area
        control_pad_area = flashButton(page)
        control_pad_area.resize(400,300)
        control_pad_area.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        control_pad_area.move(450,500)
        # control_pad_area.clicked.connect(lambda: self.gotonext())
        control_pad_area.pressed.connect(lambda: self.ros_button_callback_wrapper(2, True))
        control_pad_area.released.connect(lambda: self.ros_button_callback_wrapper(2, False))
        control_pad_area.setObjectName('control_pad_area')

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

        page.mousePressEvent = control_pad_area.flash2

#TutorialCtrlNavi2
    def TutorialCtrlNavi2(self):
        
        # init layout (not sure why we need this, but seems somehow works as a placeholder)
        index = page_list.index('TutorialCtrlNavi2')
        page = self.tutorialctrlnavi2

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#GoalPrompt1
    def GoalPrompt1(self):
        
        # init layout (not sure why we need this, but seems somehow works as a placeholder)
        index = page_list.index('GoalPrompt1')
        page = self.goalprompt1

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#GoalPrompt2
    def GoalPrompt2(self):
        
        #setimage
        index = page_list.index('GoalPrompt2')
        page = self.goalprompt2

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#GoalPrompt3
    def GoalPrompt3(self):
        
        index = page_list.index('GoalPrompt3')
        page = self.goalprompt3

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#GoalPrompt4
    def GoalPrompt4(self):
        
        #setimage
        index = page_list.index('GoalPrompt4')
        page = self.goalprompt4

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)


#GoalPrompt5
    def GoalPrompt5(self):
        
        index = page_list.index('GoalPrompt5')
        page = self.goalprompt5

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#GoalPrompt6
    def GoalPrompt6(self):
        
        #setimage
        index = page_list.index('GoalPrompt6')
        page = self.goalprompt6

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#GoalPrompt7
    def GoalPrompt7(self):
        
        #setimage
        index = page_list.index('GoalPrompt7')
        page = self.goalprompt7

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#IntroductionTitle
    def IntroductionTitle(self):
        
        #setimage
        index = page_list.index('IntroductionTitle')
        page = self.introductiontitle

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#Introduction1
    def Introduction1(self):
        
        #setimage
        index = page_list.index('Introduction1')
        page = self.introduction1

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#Introduction2
    def Introduction2(self):
        
        #setimage
        index = page_list.index('Introduction2')
        page = self.introduction2

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#Introduction3
    def Introduction3(self):
        
        #setimage
        index = page_list.index('Introduction3')
        page = self.introduction3

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#UsageCondition1
    def UsageCondition1(self):
        
        #setimage
        index = page_list.index('UsageCondition1')
        page = self.usagecondition1

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#UsageCondition2
    def UsageCondition2(self):
        
        index = page_list.index('UsageCondition2')
        page = self.usagecondition2

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#UsageCondition3
    def UsageCondition3(self):
        
        index = page_list.index('UsageCondition3')
        page = self.usagecondition3

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#UsageCondition4
    def UsageCondition4(self):
        
        index = page_list.index('UsageCondition4')
        page = self.usagecondition4

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#EngagingACCTitle
    def EngagingACCTitle(self):
        
        #setimage
        index = page_list.index('EngagingACCTitle')
        page = self.engagingtitle

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))
            
    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

        

#EngagingACC1
    def EngagingACC1(self):
        
        #setimage
        index = page_list.index('EngagingACC1')
        page = self.engagingacc1

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #cruise
        cruise = flashButton(page)
        cruise.resize(200,200)
        cruise.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        cruise.move(940,670)
        # cruise.clicked.connect(lambda: self.gotonext())
        cruise.pressed.connect(lambda: self.ros_button_callback_wrapper(5, True))
        cruise.released.connect(lambda: self.ros_button_callback_wrapper(5, False))
        cruise.setObjectName('cruise')

        page.mousePressEvent = cruise.flash2
        
    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#EngagingACC2
    def EngagingACC2(self):
        
        #setimage
        index = page_list.index('EngagingACC2')
        page = self.engagingacc2
        
        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))
    
    #thumbwheel
        thumbwheel = flashButton(page)
        thumbwheel.resize(120,200)
        thumbwheel.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        thumbwheel.move(870,730)
        # thumbwheel.clicked.connect(lambda: self.gotonext())
        thumbwheel.pressed.connect(lambda: self.ros_button_callback_wrapper(3, True))
        thumbwheel.released.connect(lambda: self.ros_button_callback_wrapper(3, False))
        thumbwheel.setObjectName('thumbwheel')

        page.mousePressEvent=thumbwheel.flash2
    
    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette) 

#EngagingACC3
    def EngagingACC3(self):
        
        #setimage
        index = page_list.index('EngagingACC3')
        page = self.engagingacc3

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))
    
    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#AdjustingSpeedTitle
    def AdjustingSpeedTitle(self):
        
        #setimage
        index = page_list.index('AdjustingSpeedTitle')
        page = self.adjustingspeedtitle

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)  

    def setclickbool(self):
        self.setclicked = True
        
    def resumeclickbool(self):
        self.resumeclicked = True

#AdjustingSpeed1
    def AdjustingSpeed1(self):
        
        #setimage
        
        index = page_list.index('AdjustingSpeed1')
        page = self.adjustingspeed1

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #thumbwheel#1
        set = flashButton(page)
        set.resize(120,200)
        set.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        set.move(500,650)
        # set.clicked.connect(lambda: self.gotonext())
        set.pressed.connect(lambda: self.ros_button_callback_wrapper(3, True))
        set.released.connect(lambda: self.ros_button_callback_wrapper(3, False))
        set.setObjectName('set')
    
    #thumbwheel#2
        resume = flashButton(page)
        resume.resize(120,200)
        resume.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        resume.move(1225,750)
        # resume.clicked.connect(lambda: self.gotonext())
        resume.pressed.connect(lambda: self.ros_button_callback_wrapper(2, True))
        resume.released.connect(lambda: self.ros_button_callback_wrapper(2, False))
        resume.setObjectName('resume')

        page.mousePressEvent=self.adjustingspeed1flash
    
    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)   

#AdjustingSpeed2
    def AdjustingSpeed2(self):
        
        #setimage
        index = page_list.index('AdjustingSpeed2')
        page = self.adjustingspeed2

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #thumbwheel
        thumbwheel = flashButton(page)
        thumbwheel.resize(100, 200)
        thumbwheel.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        thumbwheel.move(875,710)
        # thumbwheel.clicked.connect(lambda: self.gotonext())
        thumbwheel.pressed.connect(lambda: self.ros_button_callback_wrapper(3, True))
        thumbwheel.released.connect(lambda: self.ros_button_callback_wrapper(3, False))
        thumbwheel.setObjectName('thumbwheel')

        page.mousePressEvent = thumbwheel.flash2
    
    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#AdjustingSpeed3
    def AdjustingSpeed3(self):
        
        #setimage
        index = page_list.index('AdjustingSpeed3')
        page = self.adjustingspeed3

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)
        

#AdjustingSpeed4
    def AdjustingSpeed4(self):
        
        #setimage
        index = page_list.index('AdjustingSpeed4')
        page = self.adjustingspeed4

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #brake
        brake = flashButton(page)
        brake.resize(320,290)
        brake.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        brake.move(480,730)
        brake.setObjectName('brake')
        brake.pressed.connect(lambda: self.ros_brake_callback_wrapper(True))
        brake.released.connect(lambda: self.ros_brake_callback_wrapper(False))

    #thumbwheel
        thumbwheel = flashButton(page)
        thumbwheel.resize(100,200)
        thumbwheel.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        thumbwheel.move(1150,800)
        # thumbwheel.clicked.connect(lambda: self.gotonext())
        thumbwheel.pressed.connect(lambda: self.ros_button_callback_wrapper(3, True))
        thumbwheel.released.connect(lambda: self.ros_button_callback_wrapper(3, False))
        thumbwheel.setObjectName('thumbwheel')

        page.mousePressEvent = self.adjustingspeed4flash

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#TryEngageAdjust
    def TryEngageAdjust(self):
        
        #setimage
        index = page_list.index('TryEngageAdjust')
        page = self.tryengageadjust

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

        # 6: "Cancel"
        Cancel = flashButton(page)
        Cancel.resize(100,100)
        Cancel.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        Cancel.move(780,620)
        # Cancel.clicked.connect(lambda: self.gotonext()) 
        Cancel.pressed.connect(lambda: self.ros_button_callback_wrapper(6, True))
        Cancel.released.connect(lambda: self.ros_button_callback_wrapper(6, False))
        Cancel.setObjectName('Cancel')
        # 5: "Cruise On/Off"
        Cruise = flashButton(page)
        Cruise.resize(100,100)
        Cruise.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        Cruise.move(960,630)
        # Cruise.clicked.connect(lambda: self.gotonext())
        Cruise.pressed.connect(lambda: self.ros_button_callback_wrapper(5, True))
        Cruise.released.connect(lambda: self.ros_button_callback_wrapper(5, False))
        Cruise.setObjectName('Cruise')
        # 4: "SL/SW On/Off/GAP" collision
        collision = flashButton(page)
        collision.resize(100,100)
        collision.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        collision.move(1050,550)
        # collision.clicked.connect(lambda: self.gotonext())
        collision.pressed.connect(lambda: self.ros_button_callback_wrapper(4, True))
        collision.released.connect(lambda: self.ros_button_callback_wrapper(4, False))
        collision.setObjectName('collision')
        # 3: "Set" thumbwheel
        set = flashButton(page)
        set.resize(100,100)
        set.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        set.move(880,730)
        # set.clicked.connect(lambda: self.gotonext())
        set.pressed.connect(lambda: self.ros_button_callback_wrapper(3, True))
        set.released.connect(lambda: self.ros_button_callback_wrapper(3, False))
        set.setObjectName('set')
        # 2: "Resume" thumbwheel
        Resume = flashButton(page)
        Resume.resize(100,100)
        Resume.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        Resume.move(880,520)
        # Resume.clicked.connect(lambda: self.gotonext())
        Resume.pressed.connect(lambda: self.ros_button_callback_wrapper(2, True))
        Resume.released.connect(lambda: self.ros_button_callback_wrapper(2, False))
        Resume.setObjectName('Resume')
        
        page.mousePressEvent = self.tryengageadjustflash

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#Try1ReviewTitle
    def Try1ReviewTitle(self):
        
        # init layout (not sure why we need this, but seems somehow works as a placeholder)
        index = page_list.index('Try1ReviewTitle')
        page = self.try1reviewtitle

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#Try1Review1
    def Try1Review1(self):
        
        #setimage
        index = page_list.index('Try1Review1')
        page = self.try1review1

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#Try1Review2
    def Try1Review2(self):
        
        #setimage
        index = page_list.index('Try1Review2')
        page = self.try1review2

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#DisengagingACCTitle
    def DisengagingACCTitle(self):
        
        #setimage
        index = page_list.index('DisengagingACCTitle')
        page = self.disengagingacctitle

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#DisengagingACC1
    def DisengagingACC1(self):
        
        #setimage
        index = page_list.index('DisengagingACC1')
        page = self.disengagingacc1

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))
    
        #disengage
        disengage = flashButton(page)
        disengage.resize(200,200)
        disengage.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        disengage.move(680,650)
        # disengage.clicked.connect(lambda: self.gotonext())
        disengage.pressed.connect(lambda: self.ros_button_callback_wrapper(6, True))
        disengage.released.connect(lambda: self.ros_button_callback_wrapper(6, False))
        disengage.setObjectName('disengage')

        page.mousePressEvent = disengage.flash2

        #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
        #fwd_arrow
        # fwd_arrow = QPushButton(page)
        # self.fwd_arrow(fwd_arrow)
    
        #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#DisengagingACC2
    def DisengagingACC2(self):
        
        #setimage
        index = page_list.index('DisengagingACC2')
        page = self.disengagingacc2

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))
            
    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#DisengagingACC3
    def DisengagingACC3(self):
        
        #setimage
        index = page_list.index('DisengagingACC3')
        page = self.disengagingacc3
        
        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))
    
    #brake_pedal
        brake_pedal = flashButton(page)
        brake_pedal.resize(450,450)
        brake_pedal.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        brake_pedal.move(702,595)
        # brake_pedal.clicked.connect(lambda: self.gotonext())
        brake_pedal.pressed.connect(lambda: self.ros_brake_callback_wrapper(True))
        brake_pedal.released.connect(lambda: self.ros_brake_callback_wrapper(False))
        brake_pedal.setObjectName('brake_pedal')

        page.mousePressEvent = brake_pedal.flash2
    
    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#DisengagingACC4
    def DisengagingACC4(self):
        
        #setimage
        index = page_list.index('DisengagingACC4')
        page = self.disengagingacc4
        
        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#DisengagingACC5
    def DisengagingACC5(self):
        
        #setimage
        index = page_list.index('DisengagingACC5')
        page = self.disengagingacc5
        
        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#DisengagingACC6
    def DisengagingACC6(self):
        
        #setimage
        index = page_list.index('DisengagingACC6')
        page = self.disengagingacc6
        
        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #brake_pedal
        brake_pedal = flashButton(page)
        brake_pedal.resize(200,200)
        brake_pedal.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        brake_pedal.move(875,755)
        # brake_pedal.clicked.connect(lambda: self.gotonext())
        brake_pedal.pressed.connect(lambda: self.ros_button_callback_wrapper(5, True))
        brake_pedal.released.connect(lambda: self.ros_button_callback_wrapper(5, False))
        brake_pedal.setObjectName('brake_pedal')

        page.mousePressEvent = brake_pedal.flash2

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#DisengagingACC7
    def DisengagingACC7(self):
        
        #setimage
        index = page_list.index('DisengagingACC7')
        page = self.disengagingacc7
        
        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#ResumingACCTitle
    def ResumingACCTitle(self):
        
        #setimage
        index = page_list.index('ResumingACCTitle')
        page = self.resumingacctitle
        
        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#ResumingACC1
    def ResumingACC1(self):
        
        #setimage
        index = page_list.index('ResumingACC1')
        page = self.resumingacc1
        
        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #thumbwheel
        thumbwheel = flashButton(page)
        thumbwheel.resize(120,200)
        thumbwheel.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        thumbwheel.move(350,550)
        # thumbwheel.clicked.connect(lambda: self.gotonext())
        thumbwheel.pressed.connect(lambda: self.ros_button_callback_wrapper(2, True))
        thumbwheel.released.connect(lambda: self.ros_button_callback_wrapper(2, False))
        thumbwheel.setObjectName('thumbwheel')

        page.mousePressEvent = thumbwheel.flash2

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#ResumingACC2
    def ResumingACC2(self):
        
        #setimage
        index = page_list.index('ResumingACC2')
        page = self.resumingacc2
        
        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))
    
    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#ResumingACC3
    def ResumingACC3(self):
        
        #setimage
        index = page_list.index('ResumingACC3')
        page = self.resumingacc3

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#ResumingACC4
    def ResumingACC4(self):
        
        #setimage
        index = page_list.index('ResumingACC4')
        page = self.resumingacc4

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#TryDisengageAdjust
    def TryDisengageAdjust(self):
        
        #setimage
        index = page_list.index('TryDisengageAdjust')
        page = self.trydisengageadjust

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

        # 6: "Cancel"
        Cancel = flashButton(page)
        Cancel.resize(100,100)
        Cancel.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        Cancel.move(785,675)
        # Cancel.clicked.connect(lambda: self.gotonext()) 
        Cancel.pressed.connect(lambda: self.ros_button_callback_wrapper(6, True))
        Cancel.released.connect(lambda: self.ros_button_callback_wrapper(6, False))
        Cancel.setObjectName('Cancel')

        # 5: "Cruise On/Off"
        Cruise = flashButton(page)
        Cruise.resize(100,100)
        Cruise.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        Cruise.move(960,675)
        # Cruise.clicked.connect(lambda: self.gotonext())
        Cruise.pressed.connect(lambda: self.ros_button_callback_wrapper(5, True))
        Cruise.released.connect(lambda: self.ros_button_callback_wrapper(5, False))
        Cruise.setObjectName('Cruise')
        # 4: "SL/SW On/Off/GAP" collision
        collision = flashButton(page)
        collision.resize(100,100)
        collision.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        collision.move(1040,600)
        # collision.clicked.connect(lambda: self.gotonext())
        collision.pressed.connect(lambda: self.ros_button_callback_wrapper(4, True))
        collision.released.connect(lambda: self.ros_button_callback_wrapper(4, False))
        collision.setObjectName('collision')
        # 3: "Set" thumbwheel
        set = flashButton(page)
        set.resize(100,100)
        set.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        set.move(880,780)
        # set.clicked.connect(lambda: self.gotonext())
        set.pressed.connect(lambda: self.ros_button_callback_wrapper(3, True))
        set.released.connect(lambda: self.ros_button_callback_wrapper(3, False))
        set.setObjectName('set')
        # 2: "Resume" thumbwheel
        Resume = flashButton(page)
        Resume.resize(100,100)
        Resume.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        Resume.move(880,570)
        # Resume.clicked.connect(lambda: self.gotonext())
        Resume.pressed.connect(lambda: self.ros_button_callback_wrapper(2, True))
        Resume.released.connect(lambda: self.ros_button_callback_wrapper(2, False))
        Resume.setObjectName('Resume')

        page.mousePressEvent = self.trydisengageadjustflash

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#Try2ReviewTitle
    def Try2ReviewTitle(self):
        
        # init layout (not sure why we need this, but seems somehow works as a placeholder)
        index = page_list.index('Try2ReviewTitle')
        page = self.try2reviewtitle

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#Try2Review1
    def Try2Review1(self):
        
        #setimage
        index = page_list.index('Try2Review1')
        page = self.try2review1

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#Try2Review2
    def Try2Review2(self):
        
        #setimage
        index = page_list.index('Try2Review2')
        page = self.try2review2

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#SettingGapDistanceTitle
    def SettingGapDistanceTitle(self):
        
        #setimage
        index = page_list.index('SettingGapDistanceTitle')
        page = self.settinggapdistancetitle

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#SettingGapDistance1
    def SettingGapDistance1(self):
        
        #setimage
        index = page_list.index('SettingGapDistance1')
        page = self.settinggapdistance1
        
        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#SettingGapDistance2
    def SettingGapDistance2(self):
        
        #setimage
        index = page_list.index('SettingGapDistance2')
        page = self.settinggapdistance2

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))
    
    #sl/sw
        slsw = flashButton(page)
        slsw.resize(200,200)
        slsw.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        slsw.move(1005,485)
        # slsw.clicked.connect(lambda: self.gotonext())
        slsw.pressed.connect(lambda: self.ros_button_callback_wrapper(4, True))
        slsw.released.connect(lambda: self.ros_button_callback_wrapper(4, False))
        slsw.setObjectName('slsw')

        page.mousePressEvent = slsw.flash2
    
    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#SettingGapDistance3
    def SettingGapDistance3(self):
        
        #setimage
        index = page_list.index('SettingGapDistance3')
        page = self.settinggapdistance3

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))
    
    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#SwitchingfromACCtoCCTitle
    def SwitchingfromACCtoCCTitle(self):
        
        #setimage
        index = page_list.index('SwitchingfromACCtoCCTitle')
        page = self.switchingfromacctocctitle

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#SwitchingfromACCtoCC1
    def SwitchingfromACCtoCC1(self):
        
        #setimage
        index = page_list.index('SwitchingfromACCtoCC1')
        page = self.switchingfromacctocc1
        
        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #disengage
        disengage = flashButton(page)
        disengage.resize(200,200)
        disengage.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        disengage.move(655,575)
        # disengage.clicked.connect(lambda: self.gotonext())
        disengage.pressed.connect(lambda: self.ros_button_callback_wrapper(6, True))
        disengage.released.connect(lambda: self.ros_button_callback_wrapper(6, False))
        disengage.setObjectName('disengage')

        page.mousePressEvent = disengage.flash2

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#SwitchingfromACCtoCC2
    def SwitchingfromACCtoCC2(self):
        
        #setimage
        index = page_list.index('SwitchingfromACCtoCC2')
        page = self.switchingfromacctocc2
        
        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#TryGapSwitch
    def TryGapSwitch(self):
        
        #setimage
        index = page_list.index('TryGapSwitch')
        page = self.trygapswitch

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

        # 6: "Cancel"
        Cancel = flashButton(page)
        Cancel.resize(100,100)
        Cancel.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        Cancel.move(815,695)
        # Cancel.clicked.connect(lambda: self.gotonext()) 
        Cancel.pressed.connect(lambda: self.ros_button_callback_wrapper(6, True))
        Cancel.released.connect(lambda: self.ros_button_callback_wrapper(6, False))
        Cancel.setObjectName('Cancel')
        # 5: "Cruise On/Off"
        Cruise = flashButton(page)
        Cruise.resize(100,100)
        Cruise.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        Cruise.move(990,695)
        # Cruise.clicked.connect(lambda: self.gotonext())
        Cruise.pressed.connect(lambda: self.ros_button_callback_wrapper(5, True))
        Cruise.released.connect(lambda: self.ros_button_callback_wrapper(5, False))
        Cruise.setObjectName('Cruise')
        # 4: "SL/SW On/Off/GAP" collision
        collision = flashButton(page)
        collision.resize(100,100)
        collision.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        collision.move(1080,630)
        # collision.clicked.connect(lambda: self.gotonext())
        collision.pressed.connect(lambda: self.ros_button_callback_wrapper(4, True))
        collision.released.connect(lambda: self.ros_button_callback_wrapper(4, False))
        collision.setObjectName('collision')
        # 3: "Set" thumbwheel
        set = flashButton(page)
        set.resize(100,100)
        set.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        set.move(910,800)
        # set.clicked.connect(lambda: self.gotonext())
        set.pressed.connect(lambda: self.ros_button_callback_wrapper(3, True))
        set.released.connect(lambda: self.ros_button_callback_wrapper(3, False))
        set.setObjectName('set')
        # 2: "Resume" thumbwheel
        Resume = flashButton(page)
        Resume.resize(100,100)
        Resume.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        Resume.move(910,590)
        # Resume.clicked.connect(lambda: self.gotonext())
        Resume.pressed.connect(lambda: self.ros_button_callback_wrapper(2, True))
        Resume.released.connect(lambda: self.ros_button_callback_wrapper(2, False))
        Resume.setObjectName('Resume')

        page.mousePressEvent = self.trygapswitchflash

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#Try3ReviewTitle
    def Try3ReviewTitle(self):
        
        # init layout (not sure why we need this, but seems somehow works as a placeholder)
        index = page_list.index('Try3ReviewTitle')
        page = self.try3reviewtitle

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

#Try3Review
    def Try3Review(self):
        
        #setimage
        index = page_list.index('Try3Review')
        page = self.try3review

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

    #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
        
    #fwd_arrow
        fwd_arrow = QPushButton(page)
        self.fwd_arrow(fwd_arrow)
    
    #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)


#TableofContents
    def TableofContents(self):
        
        #setimage
        index = page_list.index('TableofContents')
        page = self.tableofcontents

        image  = self.displaypic(index)
        image = image.scaled(self.width(), self.height())
        palette = QPalette()
        palette.setBrush(QPalette.Window, QBrush(image))

        #introduction
        introduction = QPushButton(page)
        introduction.resize(450,100)
        introduction.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        introduction.move(300,335)
        introduction.clicked.connect(lambda: self.gotopage(page_list.index('IntroductionTitle')))

        #engagingacc
        engagingacc = QPushButton(page)
        engagingacc.resize(480,100)
        engagingacc.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        engagingacc.move(300,515)
        engagingacc.clicked.connect(lambda: self.gotopage(page_list.index('EngagingACCTitle')))

        #disengagingacc
        disengagingacc = QPushButton(page)
        disengagingacc.resize(550,100)
        disengagingacc.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        disengagingacc.move(300,685)
        disengagingacc.clicked.connect(lambda: self.gotopage(page_list.index('DisengagingACCTitle')))


        #resumingacc
        resumingacc = QPushButton(page)
        resumingacc.resize(500,100)
        resumingacc.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        resumingacc.move(1010,335)
        resumingacc.clicked.connect(lambda: self.gotopage(page_list.index('ResumingACCTitle')))

        #adjustingspeed
        adjustingspeed = QPushButton(page)
        adjustingspeed.resize(530,100)
        adjustingspeed.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        adjustingspeed.move(1010,515)
        adjustingspeed.clicked.connect(lambda: self.gotopage(page_list.index('AdjustingSpeedTitle')))

        #settinggapdistance
        settinggapdistance = QPushButton(page)
        settinggapdistance.resize(620,100)
        settinggapdistance.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        settinggapdistance.move(1010,685)
        settinggapdistance.clicked.connect(lambda: self.gotopage(page_list.index('SettingGapDistanceTitle')))

        #switchingfromacctocc
        switchingfromacctocc = QPushButton(page)
        switchingfromacctocc.resize(780,100)
        switchingfromacctocc.setStyleSheet("background-color:transparent; border: 5px solid yellow" if displayButton else "background-color:transparent; border: 0px")
        switchingfromacctocc.move(600,865)
        switchingfromacctocc.clicked.connect(lambda: self.gotopage(page_list.index('SwitchingfromACCtoCCTitle')))

        #back_arrow
        back_arrow = QPushButton(page)
        self.back_arrow(back_arrow)
    
        #exit_box
        exit_box = QPushButton(page)
        self.exit_box(exit_box)

        page.setAutoFillBackground(True)
        page.setPalette(palette)

class RosThread(QThread):
    btn_flash_signal = pyqtSignal(flashButton)
    adjustingspeed1_next_pg_signal = pyqtSignal()
    trygapswitch_next_page_signal = pyqtSignal(bool)
    def __init__(self, tutorial):
        QThread.__init__(self, None)
        self.tutorial = tutorial
        return
    def startFlash(self, target_button):
        self.btn_flash_signal.emit(target_button)

    def mouse_click(self):
        global isGapSwPressed
        global gapSwCount
        global isCancelPressed
        global isCruiseOnOffPressed
        global isSetPressed
        global isResumePressed
        global isNoButtonPressed
        global isAccelPedalPressed
        global isBrakePedalPressed
        
        page = self.tutorial.currentPage()
        #print("mouse_click called at page = " + str(page) + ", with parameters switch status = " + str(switch_status) + ", acc_status = " + str(accelerate_status) + ", brake_status = " + str(brake_status))
        # 6: "Cancel" 
        # 5: "Cruise On/Off"
        # 4: "SL/SW On/Off/GAP" collision
        # 3: "Set" thumbwheel
        # 2: "Resume" thumbwheel
        # 1: placeholder
        
        
        if page == 4:
            if isCancelPressed or isCruiseOnOffPressed or isGapSwPressed or isSetPressed or isResumePressed:
                if not isNoButtonPressed: self.tutorial.play_warning(False)
                self.adjustingspeed1_next_pg_signal.emit()
                #self.tutorial.gotonext()
            elif isBrakePedalPressed:
                self.tutorial.play_warning(False)
                self.adjustingspeed1_next_pg_signal.emit()
            else:
                if not isNoButtonPressed:
                    self.tutorial.play_warning()
                    self.startFlash(self.tutorial.tutorialctrlnavi.findChild(flashButton, 'control_pad_area'))
                    # self.tutorial.tutorialctrlnavi.findChild(flashButton, 'control_pad_area').flash(default_flash_times)

        if page == 22:
            if isCruiseOnOffPressed:
                #self.tutorial.gotonext()
                if not isNoButtonPressed: self.tutorial.play_warning(False)
                self.adjustingspeed1_next_pg_signal.emit()
            else:
                if not isNoButtonPressed:
                    self.tutorial.play_warning()
                    self.startFlash(self.tutorial.engagingacc1.findChild(flashButton, 'cruise'))
                    # tutorial.engagingacc1.findChild(flashButton, 'cruise').flash(default_flash_times)
        
        if page == 23:
            if isSetPressed:
                #self.tutorial.gotonext()
                if not isNoButtonPressed: self.tutorial.play_warning(False)
                self.adjustingspeed1_next_pg_signal.emit()
            else:
                if not isNoButtonPressed:
                    self.tutorial.play_warning()
                    self.startFlash(self.tutorial.engagingacc2.findChild(flashButton, 'thumbwheel'))
                    # tutorial.engagingacc2.findChild(flashButton, 'thumbwheel').flash(default_flash_times)
        
        if page == 26:
            if isResumePressed and isSetPressed:
                if not isNoButtonPressed: self.tutorial.play_warning(False)
                self.adjustingspeed1_next_pg_signal.emit()
            else:
                if isResumePressed:
                    if not isNoButtonPressed:
                        self.startFlash(self.tutorial.adjustingspeed1.findChild(flashButton, 'set'))
                        # tutorial.adjustingspeed1.findChild(flashButton, 'set').flash(default_flash_times)
                elif isSetPressed:
                    if not isNoButtonPressed:
                        self.startFlash(self.tutorial.adjustingspeed1.findChild(flashButton, 'resume'))
                        # tutorial.adjustingspeed1.findChild(flashButton, 'resume').flash(default_flash_times)
                else:
                    if not isNoButtonPressed:
                        self.tutorial.play_warning()
                        self.startFlash(self.tutorial.adjustingspeed1.findChild(flashButton, 'set'))
                        self.startFlash(self.tutorial.adjustingspeed1.findChild(flashButton, 'resume'))
                        # tutorial.adjustingspeed1.findChild(flashButton, 'set').flash(default_flash_times)
                        # tutorial.adjustingspeed1.findChild(flashButton, 'resume').flash(default_flash_times)

        if page == 27:
            if isSetPressed:
                #self.tutorial.gotonext()
                if not isNoButtonPressed: self.tutorial.play_warning(False)
                self.adjustingspeed1_next_pg_signal.emit()
            else:
                if not isNoButtonPressed:
                    self.tutorial.play_warning()
                    self.startFlash(self.tutorial.adjustingspeed2.findChild(flashButton, 'thumbwheel'))
                    # tutorial.adjustingspeed2.findChild(flashButton, 'thumbwheel').flash(default_flash_times)

        if page == 29:
            if isBrakePedalPressed:
                if self.tutorial.next_target==None:
                    self.tutorial.next_target='set'
                    isSetPressed = False
                    return
                if isSetPressed:
                    #self.tutorial.gotonext()
                    if not isNoButtonPressed: self.tutorial.play_warning(False)
                    self.adjustingspeed1_next_pg_signal.emit()
                else:
                    if not isNoButtonPressed:
                        self.tutorial.play_warning()
                        self.startFlash(self.tutorial.adjustingspeed4.findChild(flashButton, 'thumbwheel'))
                        # tutorial.adjustingspeed4.findChild(flashButton, 'thumbwheel').flash(default_flash_times)
            else:
                self.tutorial.play_warning()
                self.startFlash(self.tutorial.adjustingspeed4.findChild(flashButton, 'brake'))
                # tutorial.adjustingspeed4.findChild(flashButton, 'brake').flash(default_flash_times)
            

        # this might be confusing to the user without any indicator of progress
        if page == 33:
            if isCruiseOnOffPressed:
                if self.tutorial.next_target==None:
                    self.tutorial.next_target='set'
                    isSetPressed = False
                    return
                if isSetPressed:
                    if self.tutorial.next_target=='set':
                        self.tutorial.next_target='res'
                        isResumePressed = False
                        return
                    if isResumePressed:
                        #self.tutorial.gotonext()
                        if not isNoButtonPressed: self.tutorial.play_warning(False)
                        self.adjustingspeed1_next_pg_signal.emit()
                    else:
                        if not isNoButtonPressed:
                            self.tutorial.play_warning()
                            self.startFlash(self.tutorial.tryengageadjust.findChild(flashButton, 'Resume'))
                            # tutorial.tryengageadjust.findChild(flashButton, 'Resume').flash(default_flash_times)
                else:
                    if not isNoButtonPressed:
                        self.tutorial.play_warning()
                        self.startFlash(self.tutorial.tryengageadjust.findChild(flashButton, 'set'))
                        # tutorial.tryengageadjust.findChild(flashButton, 'set').flash(default_flash_times)
            else:
                if not isNoButtonPressed:
                    self.tutorial.play_warning()
                    self.startFlash(self.tutorial.tryengageadjust.findChild(flashButton, 'Cruise'))
                    # tutorial.tryengageadjust.findChild(flashButton, 'Cruise').flash(default_flash_times)

        if page == 35:
            if isCancelPressed:
                #self.tutorial.gotonext()
                if not isNoButtonPressed: self.tutorial.play_warning(False)
                self.adjustingspeed1_next_pg_signal.emit()
            else:
                if not isNoButtonPressed:
                    self.tutorial.play_warning()
                    self.startFlash(self.tutorial.disengagingacc1.findChild(flashButton, 'disengage'))
                    # tutorial.disengagingacc1.findChild(flashButton, 'disengage').flash(default_flash_times)

        if page == 37:
            if isBrakePedalPressed:
                #self.tutorial.gotonext()
                self.tutorial.play_warning(False)
                self.adjustingspeed1_next_pg_signal.emit()
            else:
                if not isNoButtonPressed:
                    self.tutorial.play_warning()
                    self.startFlash(self.tutorial.disengagingacc3.findChild(flashButton, 'brake_pedal'))
                    # tutorial.disengagingacc3.findChild(flashButton, 'brake_pedal').flash(default_flash_times)

        if page == 40:
            if isCruiseOnOffPressed:
                #self.tutorial.gotonext()
                if not isNoButtonPressed: self.tutorial.play_warning(False)
                self.adjustingspeed1_next_pg_signal.emit()
            else:
                if not isNoButtonPressed:
                    self.tutorial.play_warning()
                    self.startFlash(self.tutorial.disengagingacc6.findChild(flashButton, 'brake_pedal'))
                    # tutorial.disengagingacc6.findChild(flashButton, 'brake_pedal').flash(default_flash_times)

        if page == 43:
            if isResumePressed:
                #self.tutorial.gotonext()
                if not isNoButtonPressed: self.tutorial.play_warning(False)
                self.adjustingspeed1_next_pg_signal.emit()
            else:
                if not isNoButtonPressed:
                    self.tutorial.play_warning()
                    self.startFlash(self.tutorial.resumingacc1.findChild(flashButton, 'thumbwheel'))
                    # tutorial.resumingacc1.findChild(flashButton, 'thumbwheel').flash(default_flash_times)

        # this might be confusing to the user without any indicator of progress
        if page == 50:
            if isCancelPressed or isBrakePedalPressed or isCruiseOnOffPressed:
                if self.tutorial.next_target==None:
                    self.tutorial.next_target='res'
                    isResumePressed = False
                    return
                if isResumePressed:
                    #self.tutorial.gotonext()
                    if not isNoButtonPressed: self.tutorial.play_warning(False)
                    self.adjustingspeed1_next_pg_signal.emit()
                else:
                    if not isNoButtonPressed:
                        self.tutorial.play_warning()
                        self.startFlash(self.tutorial.trydisengageadjust.findChild(flashButton, 'Resume'))
                        # tutorial.trydisengageadjust.findChild(flashButton, 'Resume').flash(default_flash_times)
            else:
                if not isNoButtonPressed:
                    self.tutorial.play_warning()
                    self.startFlash(self.tutorial.trydisengageadjust.findChild(flashButton, 'Cancel'))
                    # tutorial.trydisengageadjust.findChild(flashButton, 'Cancel').flash(default_flash_times)

        if page == 53:
            if isGapSwPressed:
                #self.tutorial.gotonext()
                self.tutorial.play_warning(False)
                self.adjustingspeed1_next_pg_signal.emit()
            else:
                if not isNoButtonPressed:
                    self.tutorial.play_warning()
                    self.startFlash(self.tutorial.settinggapdistance2.findChild(flashButton, 'slsw'))
                    # tutorial.settinggapdistance2.findChild(flashButton, 'slsw').flash(default_flash_times)

        if page == 56:
            if isCancelPressed:
                if isNoButtonPressed:
                    isCancelPressed = False
                    self.trygapswitch_next_page_signal.emit(False)
                    self.tutorial.play_warning()
                    self.startFlash(self.tutorial.switchingfromacctocc1.findChild(flashButton, 'disengage'))
                else:
                    self.trygapswitch_next_page_signal.emit(True)
            else:
                if not isNoButtonPressed:
                    self.tutorial.play_warning()
                    self.startFlash(self.tutorial.switchingfromacctocc1.findChild(flashButton, 'disengage'))

        # this might be confusing to the user without any indicator of progress
        if page == 60:
            if isGapSwPressed:
                if gapSwCount>=3:
                    if self.tutorial.next_target==None:
                        self.tutorial.next_target='cancel'
                        isCancelPressed=False
                        return
                    if isCancelPressed:
                        if isNoButtonPressed:
                            isCancelPressed = False
                            self.trygapswitch_next_page_signal.emit(False)
                            self.tutorial.play_warning()
                            self.startFlash(self.tutorial.trygapswitch.findChild(flashButton, 'Cancel'))
                            # tutorial.trygapswitch.findChild(flashButton, 'Cancel').flash(default_flash_times)
                        else:
                            self.trygapswitch_next_page_signal.emit(True)
                    else:
                        if not isNoButtonPressed:
                            self.tutorial.play_warning()
                            self.startFlash(self.tutorial.trygapswitch.findChild(flashButton, 'Cancel'))
                            # tutorial.trygapswitch.findChild(flashButton, 'Cancel').flash(default_flash_times)
                else:
                    isGapSwPressed = False
            else:
                if not isNoButtonPressed:
                    self.tutorial.play_warning()
                    self.startFlash(self.tutorial.trygapswitch.findChild(flashButton, 'collision'))
                    # tutorial.trygapswitch.findChild(flashButton, 'collision').flash(default_flash_times)
                     

#ROS subscriber node
def callback_fcn_3(message): #message is the data received from the topic
    global isBrakePedalPressed

    isBrakePedalPressedTmp = True if message.BrkPdlPos>=5 else False

    if isBrakePedalPressed:
        return
    else:
        if isBrakePedalPressedTmp:
            isBrakePedalPressed = isBrakePedalPressedTmp
            rt.mouse_click()
        else:
            return

def callback_fcn_5(message): #message is the data received from the topic
    global isCancelPressed
    global isCruiseOnOffPressed
    global isSetPressed
    global isResumePressed
    global isNoButtonPressed
    global prevCrsSpdLmtrSwStat

    if(message.CrsSpdLmtrSwStat==4):
        return

    #print("callback_fcn_5: CrsSpdLmtrSwStat="+str(message.CrsSpdLmtrSwStat))

    if(prevCrsSpdLmtrSwStat==message.CrsSpdLmtrSwStat):
        return
    else:
        prevCrsSpdLmtrSwStat = message.CrsSpdLmtrSwStat

    if(message.CrsSpdLmtrSwStat==1 or message.CrsSpdLmtrSwStat==0):
        isNoButtonPressed=True
    else:
        isNoButtonPressed=False
    
    if(message.CrsSpdLmtrSwStat==2):
        isResumePressed=True
    elif(message.CrsSpdLmtrSwStat==3):
        isSetPressed=True
    elif(message.CrsSpdLmtrSwStat==5):
        isCruiseOnOffPressed=True
    elif(message.CrsSpdLmtrSwStat==6):
        isCancelPressed=True
    #print("Callback PCM 5 executed")
    rt.mouse_click()

def gap_sw_callback_fcn(message): #message is the data received from the topic
    global isGapSwPressed
    global gapSwCount
    global prevGapSwAct

    #print("gap_sw_callback_fcn: data="+str(message.data))
    
    if(prevGapSwAct==message.data):
        return
    else:
        prevGapSwAct = message.data

    if(message.data==1):
        isGapSwPressed=True
        gapSwCount+=1
        rt.mouse_click()

 
app = QtWidgets.QApplication(sys.argv)

rospy.init_node('hmi_node', anonymous=False) #log_level=rospy.INFO
pcm_page_number_pub = rospy.Publisher('HMI_Slide_Num', Float32, queue_size=10)

tutorial = widget()
rt = RosThread(tutorial)


rospy.Subscriber("PcmToCav5", PcmToCav5, callback_fcn_5)
rospy.Subscriber("PcmToCav3", PcmToCav3, callback_fcn_3)
rospy.Subscriber("Gap_Sw_Act", Float32, gap_sw_callback_fcn)


  


def fb(button):
    button.flash(default_flash_times)
rt.btn_flash_signal.connect(fb)

def cb2():
    if timerTwoSec.isActive():
        timerTwoSec.stop()
    try: timerTwoSec.timeout.disconnect() 
    except Exception: pass
    timerTwoSec.timeout.connect(tutorial.gotonext_timer)
    timerTwoSec.start()
rt.adjustingspeed1_next_pg_signal.connect(cb2)

def cb3(need_start):
    if need_start:
        if timer1p5Sec.isActive():
            timer1p5Sec.stop()
        try: timer1p5Sec.timeout.disconnect() 
        except Exception: pass
        timer1p5Sec.timeout.connect(tutorial.gotonext_timer1p5)
        timer1p5Sec.start()
    else:
        if timer1p5Sec.isActive():
            timer1p5Sec.stop()
rt.trygapswitch_next_page_signal.connect(cb3)

exit(app.exec_())
