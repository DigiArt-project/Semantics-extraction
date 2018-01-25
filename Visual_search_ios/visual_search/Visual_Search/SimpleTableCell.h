//
//  SimpleTableCell.h
//  Visual_Search
//
//  Created by Lirone Samoun on 26/10/2016.
//

/*!
 @class SimpleTableCell
 
 @brief Customization of the Cell for the tableView
 
 @discussion the SimpleTableCell class serves as the data model of custom cell
 In the cell, we have three values that are changeable: the thumbnail image view, the category label and the distance label. In the class, we’ll add three properties to represent these dynamic values.
 
 */

#import <UIKit/UIKit.h>

@interface SimpleTableCell : UITableViewCell

/*! @brief number of the label */
@property (nonatomic, weak) IBOutlet UILabel *numberLabel;
/*! @brief the category of the object */
@property (nonatomic, weak) IBOutlet UILabel *categoryLabel;
/*! @brief the distance of the object */
@property (nonatomic, weak) IBOutlet UILabel *distanceLabel;
/*! @brief thumbnail of the object */
@property (nonatomic, weak) IBOutlet UIImageView *thumbnailImageView;

@end
