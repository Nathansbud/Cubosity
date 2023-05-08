#pragma once

#include "interface/OrientationGroup.h"

#include <QMainWindow>
#include <QBoxLayout>
#include <QLabel>
#include "glwidget.h"

class MainWindow : public QWidget
{
    Q_OBJECT

public:
    MainWindow();
    ~MainWindow();

private:
    GLWidget *glWidget;

    // Keep layouts available to modify
    QHBoxLayout* hLayout;
    QVBoxLayout* vLayout;
    QVBoxLayout* rotLayout;

    QVBoxLayout* orientLayout;
    QPushButton* exitOrientationButton;
    QPushButton* hideButton;

    void createOrientationGroup();
    void addOrientationGroup(OrientationGroup*);
    void deactivateOrientationGroups();

    void addHeading(QBoxLayout *layout, QString text);
    void addLabel(QBoxLayout *layout, QString text);
    void addRadioButton(QBoxLayout *layout, QString text, bool value, auto function);
    void addSpinBox(QBoxLayout *layout, QString text, int min, int max, int step, int val, auto function);
    void addDoubleSpinBox(QBoxLayout *layout, QString text, double min, double max, double step, double val, int decimal, auto function);
    QPushButton* addPushButton(QBoxLayout *layout, QString text, auto function);
    void addCheckBox(QBoxLayout *layout, QString text, bool value, auto function);

    // Simplification
    void onSimplifyButtonClick();
    void onSimplifyTargetChange(int);

    void onExpandButtonClick();
    void onExpandAllButtonClick();

    // Subdivision
    void onSubdivideButtonClick();

    // Cubify
    void onCubifyButtonClick();

    // Denoising
    void onDenoiseButtonClick();
    void onDenoiseDistanceChange(double);
    void onDenoiseSig1Change(double);
    void onDenoiseSig2Change(double);

    // Debug Options
    void onAnimateCubingChecked(bool);
    void onAnimateExpandChecked(bool);
    void onSaveMeshClicked();

    int orientationsHidden = false;
};
