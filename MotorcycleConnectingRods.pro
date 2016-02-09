TEMPLATE = app
CONFIG += console c++14
CONFIG -= app_bundle
CONFIG -= qt

LIBS += `pkg-config opencv --libs`

SOURCES += main.cpp


# http://dragly.org/2013/11/05/copying-data-files-to-the-build-directory-when-working-with-qmake/
copydata.commands = $(COPY_DIR) $$PWD/rods/ $$OUT_PWD

first.depends = $(first) copydata
export(first.depends)
export(copydata.commands)
QMAKE_EXTRA_TARGETS += first copydata
