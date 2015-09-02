COMPONENT=TestSerialAppC
BUILD_EXTRA_DEPS += TestSerial.class
CLEAN_EXTRA = *.class TestSerialMsg.java

CFLAGS += -I$(TOSDIR)/lib/T2Hack

TestSerial.class: $(wildcard *.java) TestSerialMsg.java
	javac -target 1.4 -source 1.4 *.java

TestSerialMsg.java:
	mig java -target=null $(CFLAGS) -java-classname=TestSerialMsg TestSerial.h test_serial_msg -o $@

BUILD_EXTRA_DEPS += TestSerialMsg.py
TestSerialMsg.py: TestSerial.h
	mig python -target=$(PLATFORM) $(CFLAGS) -python-classname=TestSerialMsg TestSerial.h test_serial_msg -o $@

include $(MAKERULES)

