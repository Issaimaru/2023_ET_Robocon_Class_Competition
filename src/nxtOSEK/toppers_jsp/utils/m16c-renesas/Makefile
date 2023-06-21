#
#  TOPPERS/JSP Kernel
#      Toyohashi Open Platform for Embedded Real-Time Systems/
#      Just Standard Profile Kernel
# 
#  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
#                              Toyohashi Univ. of Technology, JAPAN
#  Copyright (C) 2003-2004 by Platform Development Center
#                                          RIOCH COMPANY,LTD. JAPAN
# 
#  �嵭����Ԥϡ��ʲ��� (1)��(4) �ξ�狼��Free Software Foundation 
#  �ˤ�äƸ�ɽ����Ƥ��� GNU General Public License �� Version 2 �˵�
#  �Ҥ���Ƥ���������������˸¤ꡤ�ܥ��եȥ��������ܥ��եȥ�����
#  ����Ѥ�����Τ�ޤࡥ�ʲ�Ʊ���ˤ���ѡ�ʣ�������ѡ������ۡʰʲ���
#  ���ѤȸƤ֡ˤ��뤳�Ȥ�̵���ǵ������롥
#  (1) �ܥ��եȥ������򥽡��������ɤη������Ѥ�����ˤϡ��嵭������
#      ��ɽ�����������Ѿ�浪��Ӳ�����̵�ݾڵ��꤬�����Τޤޤη��ǥ���
#      ����������˴ޤޤ�Ƥ��뤳�ȡ�
#  (2) �ܥ��եȥ������򡤥饤�֥������ʤɡ�¾�Υ��եȥ�������ȯ�˻�
#      �ѤǤ�����Ǻ����ۤ�����ˤϡ������ۤ�ȼ���ɥ�����ȡ�����
#      �ԥޥ˥奢��ʤɡˤˡ��嵭�����ɽ�����������Ѿ�浪��Ӳ���
#      ��̵�ݾڵ����Ǻܤ��뤳�ȡ�
#  (3) �ܥ��եȥ������򡤵�����Ȥ߹���ʤɡ�¾�Υ��եȥ�������ȯ�˻�
#      �ѤǤ��ʤ����Ǻ����ۤ�����ˤϡ����Τ����줫�ξ�����������
#      �ȡ�
#    (a) �����ۤ�ȼ���ɥ�����ȡ����Ѽԥޥ˥奢��ʤɡˤˡ��嵭����
#        �ɽ�����������Ѿ�浪��Ӳ�����̵�ݾڵ����Ǻܤ��뤳�ȡ�
#    (b) �����ۤη��֤��̤�������ˡ�ˤ�äơ�TOPPERS�ץ��������Ȥ�
#        ��𤹤뤳�ȡ�
#  (4) �ܥ��եȥ����������Ѥˤ��ľ��Ū�ޤ��ϴ���Ū�������뤤���ʤ�»
#      ������⡤�嵭����Ԥ����TOPPERS�ץ��������Ȥ����դ��뤳�ȡ�
# 
#  �ܥ��եȥ������ϡ�̵�ݾڤ��󶡤���Ƥ����ΤǤ��롥�嵭����Ԥ�
#  ���TOPPERS�ץ��������Ȥϡ��ܥ��եȥ������˴ؤ��ơ�����Ŭ�Ѳ�ǽ����
#  �ޤ�ơ������ʤ��ݾڤ�Ԥ�ʤ����ޤ����ܥ��եȥ����������Ѥˤ��ľ
#  ��Ū�ޤ��ϴ���Ū�������������ʤ�»���˴ؤ��Ƥ⡤������Ǥ�����ʤ���
# 
#  @(#) $Id: Makefile,v 1.3 2004/10/07 07:30:37 honda Exp $
# 

#
#  ����ѥ��륪�ץ��������
#
CFLAGS = -g -O2 -Wall
INCLUDES = -I.

#
#  MinGW����Ѥ��Ƥ���Ȥ��ˤϡ����Υ����Ȥ򳰤�
#
#CFLAGS := $(CFLAGS) -D__MINGW__

#
#  ��󥯻����ɲä���饤�֥��
#
#LIBS = -lstdc++

#
#  ���֥������ȥե�����̾�����
#
COMMON_OBJS = StdAfx.o
OFF_OBJS = m16coffset.o $(COMMON_OBJS)
VEC_OBJS = m16cvec.o  $(COMMON_OBJS)

VPATH = ./m16cvec ./m16coffset

#
#  �������åȥե�����
#
all: m16coffset m16cvec

#
#  �������ե�����Τ���ǥ��쥯�ȥ�˴ؤ������
#
vpath %.cpp $(VPATH)

#
#  m16coffset �Υ��
#
m16coffset: $(OFF_OBJS)
	$(CC) $(CFLAGS) -o m16coffset $(OFF_OBJS) $(LIBS)

#
#  m16cvec �Υ��
#
m16cvec: $(VEC_OBJS)
	$(CC) $(CFLAGS) -o m16cvec $(VEC_OBJS) $(LIBS)

#
#  ����ѥ����̤ξõ�
#
clean:
	rm -f *.o
	rm -f m16coffset.exe m16cvec.exe

cleandep:
	rm -f Makefile.depend

realclean: cleandep clean

#
#  ��¸�ط��ե����������
#
depend: cleandep $(OFF_OBJS:.o=.d) $(VEC_OBJS:.o=.d)

#
#  ��¸�ط��ե�����򥤥󥯥롼��
#
-include Makefile.depend

#
#  ����ѥ���ɥ饤�Ф����
#
CC = g++

#
#  �ǥե���ȤΥ���ѥ���롼������
#
%.o: %.cpp
	$(CC) -c $(CFLAGS) $(INCLUDES) $<

#
#  �ǥե���Ȥΰ�¸�ط������롼������
#
%.d: %.cpp
	$(CC) -M $(CFLAGS) $(INCLUDES) $< >> Makefile.depend