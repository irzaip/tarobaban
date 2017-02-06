# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mainWid.ui'
#
# Created: Thu Feb  2 15:12:15 2017
#      by: pyside-uic 0.2.15 running on PySide 1.2.1
#
# WARNING! All changes made in this file will be lost!

from PySide import QtCore, QtGui

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(532, 459)
        self.pushButton = QtGui.QPushButton(Form)
        self.pushButton.setGeometry(QtCore.QRect(230, 10, 161, 61))
        self.pushButton.setObjectName("pushButton")

        self.retranslateUi(Form)
        QtCore.QObject.connect(self.pushButton, QtCore.SIGNAL("toggled(bool)"), self.message())
        QtCore.QMetaObject.connectSlotsByName(Form)

    def message():
        self.pushButton.setText("Connected")
        print "Hello"
        

    def retranslateUi(self, Form):
        Form.setWindowTitle(QtGui.QApplication.translate("Form", "Form", None, QtGui.QApplication.UnicodeUTF8))
        self.pushButton.setText(QtGui.QApplication.translate("Form", "Not Connected", None, QtGui.QApplication.UnicodeUTF8))


if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    Form = QtGui.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())

