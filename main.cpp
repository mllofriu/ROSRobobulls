
#include "visioncomm.h"
#include <QApplication>


int main(int argc, char *argv[])
{
   VisionComm visionCommunicator;

     QApplication a(argc, argv);
     //GuiInterface::getGuiInterface()->show();


    visionCommunicator.start();

    return a.exec();
}
