//
//  SimpleTableCell.m
//  Visual_Search
//
//  Created by Lirone Samoun on 26/10/2016.
//  Copyright © 2016 Occipital. All rights reserved.
//

#import "SimpleTableCell.h"

@implementation SimpleTableCell

//The “@synthesize” keyword tells compiler to automatically generate code for accessing the properties we declared earlier
@synthesize categoryLabel = _nameLabel;
@synthesize distanceLabel = _prepTimeLabel;
@synthesize thumbnailImageView = _thumbnailImageView;
@synthesize numberLabel = _numberLabel;

- (void)awakeFromNib {
    [super awakeFromNib];
    // Initialization code
}

- (void)setSelected:(BOOL)selected animated:(BOOL)animated {
    [super setSelected:selected animated:animated];

    // Configure the view for the selected state
}

@end
