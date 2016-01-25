/********************************************************************************
** Form generated from reading UI file 'CMainWindow.ui'
**
** Created by: Qt User Interface Compiler version 5.6.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CMAINWINDOW_H
#define UI_CMAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QOpenGLWidget>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_CMainWindow
{
public:
    QAction *actionSave_workflow_as;
    QAction *actionLoad_images;
    QAction *actionSave_worflow;
    QAction *actionSave_results;
    QAction *actionSave_results_As;
    QAction *actionSettings;
    QAction *actionAbout;
    QAction *actionDefault_four_step_workflow;
    QAction *actionLoad_data_directory;
    QWidget *centralwidget;
    QVBoxLayout *verticalLayout_5;
    QHBoxLayout *horizontalLayout;
    QListWidget *listWidget;
    QTabWidget *tabWidget_2;
    QWidget *tab3D;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_3;
    QComboBox *comboBox_5;
    QSpacerItem *horizontalSpacer_2;
    QOpenGLWidget *openGLWidget;
    QWidget *tabDepthMap;
    QWidget *tabFeature;
    QGridLayout *gridLayout;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QVBoxLayout *verticalLayout_3;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout_2;
    QPushButton *pushButton;
    QLabel *label_5;
    QComboBox *comboBox;
    QGroupBox *groupBox_2;
    QVBoxLayout *verticalLayout_6;
    QPushButton *pushButton_2;
    QLabel *label_6;
    QComboBox *comboBox_2;
    QGroupBox *groupBox_3;
    QVBoxLayout *verticalLayout_7;
    QPushButton *pushButton_3;
    QLabel *label_7;
    QComboBox *comboBox_3;
    QGroupBox *groupBox_4;
    QVBoxLayout *verticalLayout_8;
    QLabel *label_8;
    QComboBox *comboBox_4;
    QSpacerItem *verticalSpacer;
    QTabWidget *tabWidget;
    QWidget *tabImages;
    QHBoxLayout *horizontalLayout_4;
    QListWidget *listWidget_2;
    QListWidget *listWidget_3;
    QWidget *tabLog;
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout_2;
    QCheckBox *checkBox;
    QCheckBox *checkBox_2;
    QCheckBox *checkBox_3;
    QCheckBox *checkBox_4;
    QSpacerItem *horizontalSpacer;
    QPlainTextEdit *plainTextEdit;
    QMenuBar *menubar;
    QMenu *menuFile;
    QMenu *menuSettings;
    QMenu *menuHelp;
    QMenu *menuWorkflow;

    void setupUi(QMainWindow *CMainWindow)
    {
        if (CMainWindow->objectName().isEmpty())
            CMainWindow->setObjectName(QStringLiteral("CMainWindow"));
        CMainWindow->resize(800, 743);
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(CMainWindow->sizePolicy().hasHeightForWidth());
        CMainWindow->setSizePolicy(sizePolicy);
        actionSave_workflow_as = new QAction(CMainWindow);
        actionSave_workflow_as->setObjectName(QStringLiteral("actionSave_workflow_as"));
        actionLoad_images = new QAction(CMainWindow);
        actionLoad_images->setObjectName(QStringLiteral("actionLoad_images"));
        actionSave_worflow = new QAction(CMainWindow);
        actionSave_worflow->setObjectName(QStringLiteral("actionSave_worflow"));
        actionSave_results = new QAction(CMainWindow);
        actionSave_results->setObjectName(QStringLiteral("actionSave_results"));
        actionSave_results_As = new QAction(CMainWindow);
        actionSave_results_As->setObjectName(QStringLiteral("actionSave_results_As"));
        actionSettings = new QAction(CMainWindow);
        actionSettings->setObjectName(QStringLiteral("actionSettings"));
        actionAbout = new QAction(CMainWindow);
        actionAbout->setObjectName(QStringLiteral("actionAbout"));
        actionDefault_four_step_workflow = new QAction(CMainWindow);
        actionDefault_four_step_workflow->setObjectName(QStringLiteral("actionDefault_four_step_workflow"));
        actionLoad_data_directory = new QAction(CMainWindow);
        actionLoad_data_directory->setObjectName(QStringLiteral("actionLoad_data_directory"));
        centralwidget = new QWidget(CMainWindow);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        centralwidget->setAutoFillBackground(false);
        verticalLayout_5 = new QVBoxLayout(centralwidget);
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        listWidget = new QListWidget(centralwidget);
        listWidget->setObjectName(QStringLiteral("listWidget"));

        horizontalLayout->addWidget(listWidget);

        tabWidget_2 = new QTabWidget(centralwidget);
        tabWidget_2->setObjectName(QStringLiteral("tabWidget_2"));
        tab3D = new QWidget();
        tab3D->setObjectName(QStringLiteral("tab3D"));
        verticalLayout = new QVBoxLayout(tab3D);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(9, 9, 9, 9);
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        comboBox_5 = new QComboBox(tab3D);
        comboBox_5->setObjectName(QStringLiteral("comboBox_5"));
        comboBox_5->setMinimumSize(QSize(120, 0));

        horizontalLayout_3->addWidget(comboBox_5);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer_2);


        verticalLayout->addLayout(horizontalLayout_3);

        openGLWidget = new QOpenGLWidget(tab3D);
        openGLWidget->setObjectName(QStringLiteral("openGLWidget"));
        openGLWidget->setMinimumSize(QSize(400, 400));

        verticalLayout->addWidget(openGLWidget);

        tabWidget_2->addTab(tab3D, QString());
        tabDepthMap = new QWidget();
        tabDepthMap->setObjectName(QStringLiteral("tabDepthMap"));
        tabWidget_2->addTab(tabDepthMap, QString());
        tabFeature = new QWidget();
        tabFeature->setObjectName(QStringLiteral("tabFeature"));
        gridLayout = new QGridLayout(tabFeature);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        scrollArea = new QScrollArea(tabFeature);
        scrollArea->setObjectName(QStringLiteral("scrollArea"));
        scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
        scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
        scrollArea->setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QStringLiteral("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 385, 425));
        scrollArea->setWidget(scrollAreaWidgetContents);

        gridLayout->addWidget(scrollArea, 0, 0, 1, 1);

        tabWidget_2->addTab(tabFeature, QString());

        horizontalLayout->addWidget(tabWidget_2);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        groupBox = new QGroupBox(centralwidget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        verticalLayout_2 = new QVBoxLayout(groupBox);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        pushButton = new QPushButton(groupBox);
        pushButton->setObjectName(QStringLiteral("pushButton"));

        verticalLayout_2->addWidget(pushButton);

        label_5 = new QLabel(groupBox);
        label_5->setObjectName(QStringLiteral("label_5"));

        verticalLayout_2->addWidget(label_5);

        comboBox = new QComboBox(groupBox);
        comboBox->setObjectName(QStringLiteral("comboBox"));

        verticalLayout_2->addWidget(comboBox);


        verticalLayout_3->addWidget(groupBox);

        groupBox_2 = new QGroupBox(centralwidget);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        verticalLayout_6 = new QVBoxLayout(groupBox_2);
        verticalLayout_6->setObjectName(QStringLiteral("verticalLayout_6"));
        pushButton_2 = new QPushButton(groupBox_2);
        pushButton_2->setObjectName(QStringLiteral("pushButton_2"));

        verticalLayout_6->addWidget(pushButton_2);

        label_6 = new QLabel(groupBox_2);
        label_6->setObjectName(QStringLiteral("label_6"));

        verticalLayout_6->addWidget(label_6);

        comboBox_2 = new QComboBox(groupBox_2);
        comboBox_2->setObjectName(QStringLiteral("comboBox_2"));

        verticalLayout_6->addWidget(comboBox_2);


        verticalLayout_3->addWidget(groupBox_2);

        groupBox_3 = new QGroupBox(centralwidget);
        groupBox_3->setObjectName(QStringLiteral("groupBox_3"));
        verticalLayout_7 = new QVBoxLayout(groupBox_3);
        verticalLayout_7->setObjectName(QStringLiteral("verticalLayout_7"));
        pushButton_3 = new QPushButton(groupBox_3);
        pushButton_3->setObjectName(QStringLiteral("pushButton_3"));

        verticalLayout_7->addWidget(pushButton_3);

        label_7 = new QLabel(groupBox_3);
        label_7->setObjectName(QStringLiteral("label_7"));

        verticalLayout_7->addWidget(label_7);

        comboBox_3 = new QComboBox(groupBox_3);
        comboBox_3->setObjectName(QStringLiteral("comboBox_3"));

        verticalLayout_7->addWidget(comboBox_3);


        verticalLayout_3->addWidget(groupBox_3);

        groupBox_4 = new QGroupBox(centralwidget);
        groupBox_4->setObjectName(QStringLiteral("groupBox_4"));
        verticalLayout_8 = new QVBoxLayout(groupBox_4);
        verticalLayout_8->setObjectName(QStringLiteral("verticalLayout_8"));
        label_8 = new QLabel(groupBox_4);
        label_8->setObjectName(QStringLiteral("label_8"));

        verticalLayout_8->addWidget(label_8);

        comboBox_4 = new QComboBox(groupBox_4);
        comboBox_4->setObjectName(QStringLiteral("comboBox_4"));

        verticalLayout_8->addWidget(comboBox_4);


        verticalLayout_3->addWidget(groupBox_4);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_3->addItem(verticalSpacer);


        horizontalLayout->addLayout(verticalLayout_3);


        verticalLayout_5->addLayout(horizontalLayout);

        tabWidget = new QTabWidget(centralwidget);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        tabImages = new QWidget();
        tabImages->setObjectName(QStringLiteral("tabImages"));
        horizontalLayout_4 = new QHBoxLayout(tabImages);
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        listWidget_2 = new QListWidget(tabImages);
        new QListWidgetItem(listWidget_2);
        new QListWidgetItem(listWidget_2);
        listWidget_2->setObjectName(QStringLiteral("listWidget_2"));

        horizontalLayout_4->addWidget(listWidget_2);

        listWidget_3 = new QListWidget(tabImages);
        listWidget_3->setObjectName(QStringLiteral("listWidget_3"));
        listWidget_3->setMinimumSize(QSize(600, 0));
        listWidget_3->setSelectionMode(QAbstractItemView::MultiSelection);
        listWidget_3->setViewMode(QListView::IconMode);

        horizontalLayout_4->addWidget(listWidget_3);

        tabWidget->addTab(tabImages, QString());
        tabLog = new QWidget();
        tabLog->setObjectName(QStringLiteral("tabLog"));
        verticalLayout_4 = new QVBoxLayout(tabLog);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        checkBox = new QCheckBox(tabLog);
        checkBox->setObjectName(QStringLiteral("checkBox"));
        checkBox->setChecked(true);

        horizontalLayout_2->addWidget(checkBox);

        checkBox_2 = new QCheckBox(tabLog);
        checkBox_2->setObjectName(QStringLiteral("checkBox_2"));
        checkBox_2->setChecked(true);

        horizontalLayout_2->addWidget(checkBox_2);

        checkBox_3 = new QCheckBox(tabLog);
        checkBox_3->setObjectName(QStringLiteral("checkBox_3"));
        checkBox_3->setChecked(true);

        horizontalLayout_2->addWidget(checkBox_3);

        checkBox_4 = new QCheckBox(tabLog);
        checkBox_4->setObjectName(QStringLiteral("checkBox_4"));
        checkBox_4->setChecked(true);

        horizontalLayout_2->addWidget(checkBox_4);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer);


        verticalLayout_4->addLayout(horizontalLayout_2);

        plainTextEdit = new QPlainTextEdit(tabLog);
        plainTextEdit->setObjectName(QStringLiteral("plainTextEdit"));
        plainTextEdit->setUndoRedoEnabled(false);
        plainTextEdit->setReadOnly(true);

        verticalLayout_4->addWidget(plainTextEdit);

        tabWidget->addTab(tabLog, QString());

        verticalLayout_5->addWidget(tabWidget);

        CMainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(CMainWindow);
        menubar->setObjectName(QStringLiteral("menubar"));
        menubar->setGeometry(QRect(0, 0, 800, 26));
        menuFile = new QMenu(menubar);
        menuFile->setObjectName(QStringLiteral("menuFile"));
        menuSettings = new QMenu(menubar);
        menuSettings->setObjectName(QStringLiteral("menuSettings"));
        menuHelp = new QMenu(menubar);
        menuHelp->setObjectName(QStringLiteral("menuHelp"));
        menuWorkflow = new QMenu(menubar);
        menuWorkflow->setObjectName(QStringLiteral("menuWorkflow"));
        CMainWindow->setMenuBar(menubar);

        menubar->addAction(menuFile->menuAction());
        menubar->addAction(menuWorkflow->menuAction());
        menubar->addAction(menuSettings->menuAction());
        menubar->addAction(menuHelp->menuAction());
        menuFile->addAction(actionSave_worflow);
        menuFile->addAction(actionSave_workflow_as);
        menuFile->addSeparator();
        menuFile->addAction(actionSave_results);
        menuFile->addAction(actionSave_results_As);
        menuFile->addSeparator();
        menuFile->addAction(actionLoad_images);
        menuFile->addAction(actionLoad_data_directory);
        menuSettings->addAction(actionSettings);
        menuHelp->addAction(actionAbout);
        menuWorkflow->addAction(actionDefault_four_step_workflow);

        retranslateUi(CMainWindow);

        tabWidget_2->setCurrentIndex(2);
        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(CMainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *CMainWindow)
    {
        CMainWindow->setWindowTitle(QApplication::translate("CMainWindow", "MainWindow", 0));
        actionSave_workflow_as->setText(QApplication::translate("CMainWindow", "Save workflow As...", 0));
        actionLoad_images->setText(QApplication::translate("CMainWindow", "Load images...", 0));
        actionSave_worflow->setText(QApplication::translate("CMainWindow", "Save worflow", 0));
        actionSave_results->setText(QApplication::translate("CMainWindow", "Save results", 0));
        actionSave_results_As->setText(QApplication::translate("CMainWindow", "Save results As...", 0));
        actionSettings->setText(QApplication::translate("CMainWindow", "Settings...", 0));
        actionAbout->setText(QApplication::translate("CMainWindow", "About...", 0));
        actionDefault_four_step_workflow->setText(QApplication::translate("CMainWindow", "Four-step (default)", 0));
        actionLoad_data_directory->setText(QApplication::translate("CMainWindow", "Advanced load files...", 0));
        comboBox_5->clear();
        comboBox_5->insertItems(0, QStringList()
         << QApplication::translate("CMainWindow", "Point cloud", 0)
         << QApplication::translate("CMainWindow", "Mesh", 0)
         << QApplication::translate("CMainWindow", "Textured", 0)
        );
        tabWidget_2->setTabText(tabWidget_2->indexOf(tab3D), QApplication::translate("CMainWindow", "3D View", 0));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tabDepthMap), QApplication::translate("CMainWindow", "Depth map", 0));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tabFeature), QApplication::translate("CMainWindow", "Feature", 0));
        groupBox->setTitle(QApplication::translate("CMainWindow", "Feature extraction/matching", 0));
        pushButton->setText(QApplication::translate("CMainWindow", "Start here", 0));
        label_5->setText(QApplication::translate("CMainWindow", "Finished", 0));
        comboBox->clear();
        comboBox->insertItems(0, QStringList()
         << QApplication::translate("CMainWindow", "SIFT/RANSAC", 0)
        );
        groupBox_2->setTitle(QApplication::translate("CMainWindow", "Pose estimation", 0));
        pushButton_2->setText(QApplication::translate("CMainWindow", "Start here", 0));
        label_6->setText(QApplication::translate("CMainWindow", "Running...", 0));
        comboBox_2->clear();
        comboBox_2->insertItems(0, QStringList()
         << QApplication::translate("CMainWindow", "8-point-algorithm/SVD", 0)
        );
        groupBox_3->setTitle(QApplication::translate("CMainWindow", "Depth estimation", 0));
        pushButton_3->setText(QApplication::translate("CMainWindow", "Start here", 0));
        label_7->setText(QApplication::translate("CMainWindow", "Not started yet", 0));
        comboBox_3->clear();
        comboBox_3->insertItems(0, QStringList()
         << QApplication::translate("CMainWindow", "Stereo rectification", 0)
        );
        groupBox_4->setTitle(QApplication::translate("CMainWindow", "3d fusion", 0));
        label_8->setText(QApplication::translate("CMainWindow", "Not started yet", 0));
        comboBox_4->clear();
        comboBox_4->insertItems(0, QStringList()
         << QApplication::translate("CMainWindow", "Kinect-fusion algorithm", 0)
        );

        const bool __sortingEnabled = listWidget_2->isSortingEnabled();
        listWidget_2->setSortingEnabled(false);
        QListWidgetItem *___qlistwidgetitem = listWidget_2->item(0);
        ___qlistwidgetitem->setText(QApplication::translate("CMainWindow", "Dataset 1", 0));
        QListWidgetItem *___qlistwidgetitem1 = listWidget_2->item(1);
        ___qlistwidgetitem1->setText(QApplication::translate("CMainWindow", "Dataset 2", 0));
        listWidget_2->setSortingEnabled(__sortingEnabled);

        tabWidget->setTabText(tabWidget->indexOf(tabImages), QApplication::translate("CMainWindow", "Images", 0));
        checkBox->setText(QApplication::translate("CMainWindow", "Debug", 0));
        checkBox_2->setText(QApplication::translate("CMainWindow", "Info", 0));
        checkBox_3->setText(QApplication::translate("CMainWindow", "Warning", 0));
        checkBox_4->setText(QApplication::translate("CMainWindow", "Error", 0));
        tabWidget->setTabText(tabWidget->indexOf(tabLog), QApplication::translate("CMainWindow", "Log", 0));
        menuFile->setTitle(QApplication::translate("CMainWindow", "File", 0));
        menuSettings->setTitle(QApplication::translate("CMainWindow", "Options", 0));
        menuHelp->setTitle(QApplication::translate("CMainWindow", "Help", 0));
        menuWorkflow->setTitle(QApplication::translate("CMainWindow", "Workflow", 0));
    } // retranslateUi

};

namespace Ui {
    class CMainWindow: public Ui_CMainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CMAINWINDOW_H
