#include "studio_state_manager.hpp"

#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>

int main(int argc, char* argv[]) {
    QGuiApplication application(argc, argv);
    QGuiApplication::setApplicationName(QStringLiteral("RSS1U Kinematics Studio"));
    QGuiApplication::setOrganizationName(QStringLiteral("XJTU"));

    rss1u::studio::StudioStateManager state_manager;
    QQmlApplicationEngine engine;
    engine.rootContext()->setContextProperty(QStringLiteral("studio"), &state_manager);
    QObject::connect(
        &engine,
        &QQmlApplicationEngine::objectCreationFailed,
        &application,
        [] { QCoreApplication::exit(-1); },
        Qt::QueuedConnection);
    engine.loadFromModule(QStringLiteral("RSS1UStudio"), QStringLiteral("Main"));
    return application.exec();
}
