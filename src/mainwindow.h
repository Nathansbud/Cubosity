#pragma once

#include <QMainWindow>
#include <QBoxLayout>
#include "glwidget.h"

class MainWindow : public QWidget
{
    Q_OBJECT

public:
    MainWindow();
    ~MainWindow();

private:
    GLWidget *glWidget;

    void addHeading(QBoxLayout *layout, QString text);
    void addLabel(QBoxLayout *layout, QString text);
    void addRadioButton(QBoxLayout *layout, QString text, bool value, auto function);
    void addSpinBox(QBoxLayout *layout, QString text, int min, int max, int step, int val, auto function);
    void addDoubleSpinBox(QBoxLayout *layout, QString text, double min, double max, double step, double val, int decimal, auto function);
    void addPushButton(QBoxLayout *layout, QString text, auto function);
    void addCheckBox(QBoxLayout *layout, QString text, bool value, auto function);

    // Subdivision
    void onSubdivideButtonClick();

    // Denoising
    void onDenoiseButtonClick();
    void onDenoiseDistanceChange(double);
    void onDenoiseSig1Change(double);
    void onDenoiseSig2Change(double);
};
