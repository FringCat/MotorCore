	.cpu cortex-m4
	.arch armv7e-m
	.fpu fpv4-sp-d16
	.eabi_attribute 27, 1
	.eabi_attribute 28, 1
	.eabi_attribute 20, 1
	.eabi_attribute 21, 1
	.eabi_attribute 23, 3
	.eabi_attribute 24, 1
	.eabi_attribute 25, 1
	.eabi_attribute 26, 1
	.eabi_attribute 30, 4
	.eabi_attribute 34, 1
	.eabi_attribute 18, 4
	.file	"off.c"
	.text
	.section	.rodata.str1.1,"aMS",%progbits,1
.LC0:
	.ascii	"motor=%zu MotorConfig=%zu MotorDrv=%zu MotorAlg=%zu"
	.ascii	" MotorData=%zu\012\000"
.LC1:
	.ascii	"IA=%zu Iq=%zu angle=%zu angle_el=%zu\012\000"
	.section	.text.startup,"ax",%progbits
	.align	1
	.global	main
	.syntax unified
	.thumb
	.thumb_func
	.type	main, %function
main:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, lr}
	mov	r3, #660
	movw	r2, #1044
	strd	r3, r2, [sp]
	mov	r1, #1144
	mov	r3, #608
	movs	r2, #24
	ldr	r0, .L2
	bl	printf
	movs	r3, #64
	str	r3, [sp]
	movs	r2, #20
	movs	r3, #56
	movs	r1, #0
	ldr	r0, .L2+4
	bl	printf
	movs	r0, #0
	add	sp, sp, #12
	@ sp needed
	ldr	pc, [sp], #4
.L3:
	.align	2
.L2:
	.word	.LC0
	.word	.LC1
	.size	main, .-main
	.ident	"GCC: (15:14.2.rel1-1) 14.2.1 20241119"
