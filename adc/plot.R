#! /usr/bin/Rscript
vals<-read.table("vals.tsv")
factor<-3.3/4000
plot(vals, type="l", xlab="Zeit", ylab="Wert")
pdf(file="vals.pdf")
plot(vals[,1], vals[,2]*factor, type="l", xlab="Zeit", ylab="Spannung")
dev.off()

