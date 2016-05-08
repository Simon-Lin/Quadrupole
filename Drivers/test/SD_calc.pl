#!/usr/bin/perl -w

# standard deviation calculater for files ax..gz generated by MPU6050_example_1

foreach (@ARGV) {
    open (my $FILE, "<", $_) or die "failed to open file $ARGV[0]";
    my @DATA = <$FILE>;
    close ($FILE);
    
    # mean
    my $mean = 0;
    my $n = 0;
    foreach (@DATA) {
	if ($_ =~ m/^(.+)\s(.+)$/) {
	    $mean += $2;
	    $n++;
	}
    }
    $mean /= $n;
    
    # std
    my $sd = 0;
    foreach (@DATA) {
	if ($_ =~ m/^(.+)\s(.+)$/) {
	    $sd += ($2-$mean)**2;
	}
    }
    $sd /= $n-1;

    print "statistic of file $_:\nsamples: $n\nmean: $mean\nsd^2: $sd\n\n";
}

