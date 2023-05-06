#include "mainwindow.h"

#include "interface/OrientationGroup.h"
#include "interface/MatrixInput.h"

#include <QMainWindow>
#include <QSlider>
#include <QSpinBox>
#include <QRadioButton>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QBoxLayout>
#include <QGroupBox>
#include <QColorDialog>
#include <QRandomGenerator>

void placeholder() {}
MainWindow::MainWindow()
{
    glWidget = new GLWidget();

    hLayout = new QHBoxLayout(); // horizontal layout for canvas and controls panel
    vLayout = new QVBoxLayout(); // vertical layout for control panel
    rotLayout = new QVBoxLayout(); // vertical layout for control panel

    vLayout->setAlignment(Qt::AlignTop);
    rotLayout->setAlignment(Qt::AlignTop);

    hLayout->addLayout(vLayout);
    // Force ARAP frame to occupy maximum width
    hLayout->addWidget(glWidget, 1);
    hLayout->addLayout(rotLayout);

    this->setLayout(hLayout);

    addHeading(vLayout, "Controls");

    QGroupBox* simpBox = new QGroupBox("Simplify");
    QHBoxLayout* simpLayout = new QHBoxLayout();

    addSpinBox(simpLayout, "", 4, 10000000, 1, glWidget->settings.simplifyTarget, &MainWindow::onSimplifyTargetChange);
    addPushButton(simpLayout, "Simplify", &MainWindow::onSimplifyButtonClick);
    addPushButton(simpLayout, "Expand", &MainWindow::onExpandButtonClick);
    addPushButton(simpLayout, "Expand All", &MainWindow::onExpandAllButtonClick);

    simpBox->setLayout(simpLayout);

    QGroupBox* denoiseBox = new QGroupBox("Denoise");
    QVBoxLayout* denoiseLayout = new QVBoxLayout();

    addPushButton(denoiseLayout, "Denoise", &MainWindow::onDenoiseButtonClick);
    addDoubleSpinBox(denoiseLayout, "Distance", 0.5, 10, 0.1, 2, 1, &MainWindow::onDenoiseDistanceChange);
    addDoubleSpinBox(denoiseLayout, "Gaussian [c]", 0.5, 10, 0.1, 1, 1, &MainWindow::onDenoiseSig1Change);
    addDoubleSpinBox(denoiseLayout, "Gaussian [s]", 0.5, 10, 0.1, 1, 1, &MainWindow::onDenoiseSig2Change);

    denoiseBox->setLayout(denoiseLayout);

    addPushButton(vLayout, "Subdivide", &MainWindow::onSubdivideButtonClick);
    addPushButton(vLayout, "Cubify",  &MainWindow::onCubifyButtonClick);

    vLayout->addWidget(simpBox);
    vLayout->addWidget(denoiseBox);

    QGroupBox* debugBox = new QGroupBox("Debug Controls");
    QVBoxLayout* debugLayout = new QVBoxLayout();

    addCheckBox(debugLayout, "Animate Cubing? ", glWidget->settings.animateCubing, &MainWindow::onAnimateCubingChecked);
    addCheckBox(debugLayout, "Animate Expanding? ", glWidget->settings.animateExpand, &MainWindow::onAnimateExpandChecked);

    debugBox->setLayout(debugLayout);
    vLayout->addWidget(debugBox);

    QGroupBox* orientBox = new QGroupBox("Orientations");

    orientLayout = new QVBoxLayout();
    orientBox->setLayout(orientLayout);

    addPushButton(orientLayout, "Add Group", &MainWindow::createOrientationGroup);
    OrientationGroup* baseOrientation = new OrientationGroup(QColor::fromRgb(255, 0, 0));    

    addOrientationGroup(baseOrientation);
    rotLayout->addWidget(orientBox);

    this->showFullScreen();
}

MainWindow::~MainWindow() {
    delete glWidget;
}

void MainWindow::onSimplifyButtonClick() { glWidget->simplify(); }
void MainWindow::onSimplifyTargetChange(int f) { glWidget->settings.simplifyTarget = f; }

void MainWindow::onExpandButtonClick() { glWidget->expand(); }
void MainWindow::onExpandAllButtonClick() { glWidget->expandAll(); }

void MainWindow::onSubdivideButtonClick() { glWidget->subdivide(); }

void MainWindow::onCubifyButtonClick() { glWidget->cubify(); }

void MainWindow::onDenoiseButtonClick() { glWidget->denoise(); }
void MainWindow::onDenoiseDistanceChange(double d) { glWidget->settings.denoiseDistance = d; }
void MainWindow::onDenoiseSig1Change(double s) { glWidget->settings.denoiseSigma1 = s; }
void MainWindow::onDenoiseSig2Change(double s) { glWidget->settings.denoiseSigma2 = s; }

void MainWindow::createOrientationGroup() {
    OrientationGroup* ng = new OrientationGroup(QColor::fromRgb(rand() % 255 + 1, rand() % 255 + 1, rand() % 255 + 1));
    addOrientationGroup(ng);
}

void MainWindow::addOrientationGroup(OrientationGroup* ng) {
    glWidget->settings.orientationGroups.insert({ng->groupID, ng});
    orientLayout->addWidget(ng);
}

void MainWindow::onAnimateCubingChecked(bool b) { glWidget->settings.animateCubing = b; }
void MainWindow::onAnimateExpandChecked(bool b) { glWidget->settings.animateExpand = b; }

void MainWindow::addHeading(QBoxLayout* layout, QString text) {
    QFont font;
    font.setPointSize(16);
    font.setBold(true);

    QLabel* label = new QLabel(text);
    label->setFont(font);
    layout->addWidget(label);
}

void MainWindow::addLabel(QBoxLayout *layout, QString text) {
    layout->addWidget(new QLabel(text));
}

void MainWindow::addRadioButton(QBoxLayout* layout, QString text, bool value, auto function) {
    QRadioButton* button = new QRadioButton(text);
    button->setChecked(value);
    layout->addWidget(button);
    connect(button, &QRadioButton::clicked, this, function);
}

void MainWindow::addSpinBox(QBoxLayout* layout, QString text, int min, int max, int step, int val, auto function) {
    QSpinBox* box = new QSpinBox();
    box->setMinimum(min);
    box->setMaximum(max);
    box->setSingleStep(step);
    box->setValue(val);
    QHBoxLayout *subLayout = new QHBoxLayout();
    addLabel(subLayout, text);
    subLayout->addWidget(box);
    layout->addLayout(subLayout);
    connect(box, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, function);
}

void MainWindow::addDoubleSpinBox(QBoxLayout* layout, QString text, double min, double max, double step, double val, int decimal, auto function) {
    QDoubleSpinBox* box = new QDoubleSpinBox();
    box->setMinimum(min);
    box->setMaximum(max);
    box->setSingleStep(step);
    box->setValue(val);
    box->setDecimals(decimal);
    QHBoxLayout* subLayout = new QHBoxLayout();
    addLabel(subLayout, text);
    subLayout->addWidget(box);
    layout->addLayout(subLayout);
    connect(box, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, function);
}

void MainWindow::addPushButton(QBoxLayout* layout, QString text, auto function) {
    QPushButton* button = new QPushButton(text);
    layout->addWidget(button);
    connect(button, &QPushButton::clicked, this, function);
}

void MainWindow::addCheckBox(QBoxLayout* layout, QString text, bool val, auto function) {
    QCheckBox* box = new QCheckBox(text);
    box->setChecked(val);
    layout->addWidget(box);
    connect(box, &QCheckBox::clicked, this, function);
}
