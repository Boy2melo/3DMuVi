#ifndef _H_CFOURPHASEWORKFLOW
#define _H_CFOURPHASEWORKFLOW

#include <workflow/workflow/aworkflow.h>
#include "workflow/plugin/iplugin.h"
#include "workflow/workflow/ccontextdatastore.h"

class CFourPhaseWorkflow : public AWorkflow {
private:
    IPlugin **mPlugins;

    private slots:
    // ReSharper disable once CppFunctionIsNotImplemented
    void SlotAlgorithmFinished(CContextDataStore *);

public:
    CFourPhaseWorkflow();
    ~CFourPhaseWorkflow();

    /*!
    \brief Die Anzahl an verf�gbaren Ausf�hrungsschritten bzw. Algorithmenslots
    */
    quint32 getStepCount() const override;

    /*!
    \brief Gibt den erforderlichen typ an plugin f�r einen gegebenen Slot zur�ck
    \param step Der schritt f�r den der Algorithmus zur�ckgegeben werden soll
    */
    QString getAlgorithmType(const quint32 step) const override;

    /*!
    \brief Versucht ein Plugin einem Schritt zuzuweisen
    \param step Der Schritt, dem das Plugin zugewiesen werden soll
    \param plugin Das Plugin, dass dem Schritt zugewiesen werden soll
    \return True, falls der Typ des Plugins zum Schritt passt und das Plugin gesetzt wurde
    */
    bool trySetStep(const quint32 step, IPlugin* plugin) override;

    /*!
    \brief Gibt das Plugin an einem gegebenen Schritt zur�ck
    */
    IPlugin* getStep(const quint32 step) const override;
   
    /*!
    \brief Pr�fe, ob alle ben�tigten Daten f�r die Algorithmen durch den Workflow bereitgestellt werden
    */
    bool checkAvailableDataTypes() const override;

protected:
    void executeAlgorithm(CContextDataStore* store) override;
};

#endif
